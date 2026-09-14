/**
 * gw_image.c - implementation of the process image.
 *
 * See gw_image.h for what this is for and the ownership rule that keeps it
 * correct. This file is deliberately dull: bounds checks, memcpy, and one
 * layout loop. All the interesting decisions are in the headers.
 */
#include "gw_image.h"

#include <string.h>

#include "main.h" /* HAL_GetTick */

/* ==========================================================================
 * The arena
 * ==========================================================================
 * One flat byte array, cut into regions at init. aligned(4) so that every
 * region base - and therefore every value and timestamp slot - is 32-bit
 * aligned, which is what makes stores atomic and lets the scanner and the link
 * run without a lock between them.
 *
 * It is a plain static, so it lands in .bss in RAM at 0x20000000 where DMA can
 * reach it. Do not move it to CCM (0x10000000): the STM32F4 DMA controllers
 * cannot see CCM at all, and a DMA read from there fails silently.
 */
static uint8_t gImage[GW_IMAGE_BYTES] __attribute__((aligned(4)));

/* Wire-format descriptors, exactly as RD_META sends them. */
static GwRegionDesc gDesc[GW_REGION_COUNT];

/* Absolute base of each region inside gImage. Deliberately NOT part of
 * GwRegionDesc: the ESP32 addresses by (region, offset) and must never learn an
 * absolute address, or resizing a region would become a protocol change. */
static uint16_t gBase[GW_REGION_COUNT];

static uint8_t gRegionCount;
static uint16_t gTotalBytes;
static bool gReady;

/* Index into gDesc/gBase by region id, 0xFF when the id is not configured. */
static uint8_t gIndexOf[256];

/* ==========================================================================
 * Static region table
 * ==========================================================================
 * Edit this to add or resize a region, then bump GW_CONFIG_CRC32 in
 * gw_link_cfg.h so a mismatched ESP32 tag map is detectable.
 *
 * From phase P5 this table is built at boot from the config blob in flash
 * instead of being compiled in, which is what lets a PC tool re-commission a
 * device without reflashing either chip. The runtime layout code below already
 * works that way - it loops over a table - so that change replaces this array
 * and touches nothing else in the file.
 */
typedef struct {
    uint8_t region;
    uint16_t slots;   /* structured region: slot count (rawBytes must be 0)  */
    uint16_t rawBytes;/* raw region: byte count (slots must be 0)            */
    uint8_t flags;    /* GW_REGF_*, minus ENABLED which is added below       */
} GwRegionCfg;

static const GwRegionCfg kRegionCfg[] = {
    /* SYS is raw: a diagnostics struct at the fixed offsets in gw_model.h,
     * not a set of channels. Read-only from the network side. */
    {GW_REGION_SYS, 0u, GW_SYS_SIZE, GW_REGF_RAW},

    {GW_REGION_LOCAL_IO, GW_SLOTS_LOCAL_IO, 0u, 0u},
    {GW_REGION_MB_RTU, GW_SLOTS_MB_RTU, 0u, 0u},

    /* MB_TCP is the one region the ESP32 may write. The Modbus TCP master runs
     * on the network half (it already has lwIP; putting a second TCP stack on
     * the industrial half doubles the firmware that was supposed to stay small
     * enough to review). The catch is that if the ESP32 reboots, this region
     * freezes holding plausible values - which recreates, in a new place,
     * exactly the "stale data reported as good" bug v2 exists to kill. The
     * per-region write watchdog that forces it to COMM_FAIL lands with the
     * driver in P7; until then the region is empty and reads as UNKNOWN. */
    {GW_REGION_MB_TCP, GW_SLOTS_MB_TCP, 0u, GW_REGF_WRITABLE | GW_REGF_EXTERNAL},

    {GW_REGION_CAN, GW_SLOTS_CAN, 0u, 0u},
    {GW_REGION_SERIAL, GW_SLOTS_SERIAL, 0u, 0u},
    {GW_REGION_VIRTUAL, GW_SLOTS_VIRTUAL, 0u, 0u},

    /* Event ring, sized to zero until P8. A zero-length region is still
     * described by RD_META - the ESP32 sees byteLen 0 and knows not to ask,
     * which is a cleaner answer than pretending the region does not exist. */
    {GW_REGION_EVENTS, 0u, GW_BYTES_EVENTS, GW_REGF_RAW},
};

#define REGION_CFG_COUNT (sizeof(kRegionCfg) / sizeof(kRegionCfg[0]))

/* ==========================================================================
 * Internal helpers
 * ==========================================================================
 */

static inline uint16_t alignUp4(uint16_t v) {
    return (uint16_t)((v + 3u) & (uint16_t)~3u);
}

static const GwRegionDesc* descOf(uint8_t region) {
    uint8_t idx = gIndexOf[region];
    return (idx == 0xFFu) ? (const GwRegionDesc*)0 : &gDesc[idx];
}

static uint8_t* baseOf(uint8_t region) {
    uint8_t idx = gIndexOf[region];
    return (idx == 0xFFu) ? (uint8_t*)0 : &gImage[gBase[idx]];
}

/**
 * Resolves a slot to its three storage addresses.
 * Returns GW_ST_OK, or BAD_REGION / RANGE / NOT_WRITABLE-style refusal codes.
 */
static uint8_t slotPtrs(uint8_t region, uint16_t slot, uint32_t** value, uint32_t** stamp,
                        uint8_t** qual) {
    const GwRegionDesc* d = descOf(region);
    uint8_t* base;

    if (d == 0) return GW_ST_BAD_REGION;
    if ((d->flags & GW_REGF_RAW) != 0u) return GW_ST_BAD_REGION; /* no slots here */
    if (slot >= d->slots) return GW_ST_RANGE;

    base = baseOf(region);
    if (value != 0) *value = (uint32_t*)(void*)(base + d->valueOff + (uint32_t)slot * GW_SLOT_VALUE_BYTES);
    if (stamp != 0) *stamp = (uint32_t*)(void*)(base + d->stampOff + (uint32_t)slot * GW_SLOT_STAMP_BYTES);
    if (qual != 0) *qual = base + d->qualOff + slot;
    return GW_ST_OK;
}

/* ==========================================================================
 * Lifecycle
 * ==========================================================================
 */

bool GwImage_Init(void) {
    uint16_t cursor = 0u;
    uint8_t i;

    gReady = false;
    gRegionCount = 0u;
    gTotalBytes = 0u;
    memset(gIndexOf, 0xFF, sizeof(gIndexOf));
    memset(gImage, 0, sizeof(gImage));

    /* DMA reachability. On the F407 the CCM starts at 0x10000000 and no DMA
     * controller can address it. If a future linker script ever puts .bss
     * there, every link read would return zeros with no error anywhere - so
     * refuse to start instead. */
    if (((uint32_t)(void*)gImage & 0xFFF00000u) == 0x10000000u) {
        return false;
    }

    for (i = 0u; i < (uint8_t)REGION_CFG_COUNT; i++) {
        const GwRegionCfg* c = &kRegionCfg[i];
        GwRegionDesc* d = &gDesc[gRegionCount];
        uint16_t bytes;

        if (c->slots != 0u) {
            /* Structured: three parallel blocks, value and stamp first so the
             * stamp block is 4-aligned for any slot count. */
            bytes = (uint16_t)(c->slots * GW_SLOT_TOTAL_BYTES);
            d->slots = c->slots;
            d->valueOff = 0u;
            d->stampOff = (uint16_t)(c->slots * GW_SLOT_VALUE_BYTES);
            d->qualOff = (uint16_t)(c->slots * (GW_SLOT_VALUE_BYTES + GW_SLOT_STAMP_BYTES));
        } else {
            bytes = c->rawBytes;
            d->slots = 0u;
            d->valueOff = 0u;
            d->stampOff = 0u;
            d->qualOff = 0u;
        }

        if ((uint32_t)cursor + bytes > GW_IMAGE_BYTES) {
            /* Refuse rather than truncate. A truncated last region would serve
             * plausible bytes from whatever followed it in RAM. */
            return false;
        }

        d->region = c->region;
        d->flags = (uint8_t)(c->flags | GW_REGF_ENABLED);
        d->byteLen = bytes;

        gBase[gRegionCount] = cursor;
        gIndexOf[c->region] = gRegionCount;
        gRegionCount++;

        cursor = alignUp4((uint16_t)(cursor + bytes));
    }

    gTotalBytes = cursor;

    /* Every structured slot starts UNKNOWN, not GOOD. "Never read since boot"
     * and "read successfully as zero" are different things, and the cloud has
     * to be able to tell them apart on the very first publish. */
    for (i = 0u; i < gRegionCount; i++) {
        const GwRegionDesc* d = &gDesc[i];
        if (d->slots != 0u) {
            memset(&gImage[gBase[i] + d->qualOff], GW_Q_UNKNOWN, d->slots);
        }
    }

    /* Static half of the SYS region. The counters are filled by the link. */
    gReady = true;
    GwImage_SysSetU32(GW_SYS_OFF_CONFIG_CRC, GW_CONFIG_CRC32);
    GwImage_SysSetU16(GW_SYS_OFF_CONFIG_VER, GW_CONFIG_VERSION);
    GwImage_SysSetU8(GW_SYS_OFF_PROTO_VER, (uint8_t)GW_PROTO_VERSION);
    GwImage_SysSetU8(GW_SYS_OFF_FW_MAJOR, GW_FW_MAJOR);
    GwImage_SysSetU8(GW_SYS_OFF_FW_MINOR, GW_FW_MINOR);
    GwImage_SysSetU8(GW_SYS_OFF_FW_PATCH, GW_FW_PATCH);
    GwImage_SysSetU32(GW_SYS_OFF_FAULT_BITS, GW_FAULT_NONE);

    return true;
}

/* --------------------------------------------------------------------------
 * Demo animation - bench aid, see GW_IMAGE_DEMO in gw_link_cfg.h
 * --------------------------------------------------------------------------
 * Gives the ESP32 something that visibly moves before any field bus exists, and
 * exercises every quality code the consumer has to handle. Delete the call, not
 * the understanding: when the RS-485 scanner owns MB_RTU this must be off, or
 * two writers will fight over the same slots.
 */
#if GW_IMAGE_DEMO
static void demoStep(void) {
    static uint32_t nextMs;
    static uint32_t counter;
    static uint16_t triangle;
    static int16_t direction = 1;
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - nextMs) < 0) return;
    nextMs = now + 100u; /* 10 Hz is fast enough to see, slow enough to read */

    counter++;
    triangle = (uint16_t)(triangle + (uint16_t)(direction * 25));
    if (triangle > 1000u) direction = -1;
    if (triangle < 25u) direction = 1;

    /* slot 0: free-running counter - proves the image is being refreshed */
    GwImage_SetU32(GW_REGION_MB_RTU, 0u, counter, GW_Q_GOOD);
    /* slot 1: triangle wave - proves values change coherently */
    GwImage_SetU32(GW_REGION_MB_RTU, 1u, triangle, GW_Q_GOOD);
    /* slot 2: float, to prove the 4-byte slot carries a bit pattern untouched */
    GwImage_SetF32(GW_REGION_MB_RTU, 2u, 3.14159f + (float)(triangle) * 0.001f, GW_Q_GOOD);
    /* slot 3: boolean, toggling once a second */
    GwImage_SetU32(GW_REGION_MB_RTU, 3u, ((now / 1000u) & 1u), GW_Q_GOOD);

    /* slot 4: a device that answered once and then stopped.
     *
     * Written with a real value on the first pass, then only ever marked
     * COMM_FAIL. The last known value and its timestamp stay visible - useful
     * to a human, misleading on a chart - while the quality byte says not to
     * trust it. This is exactly the case link v1 reported to the cloud as
     * STATUS_OK with stale bytes, and reproducing it here means the consumer
     * side gets tested against it before a real device ever fails. */
    if (counter == 1u) {
        GwImage_SetU32(GW_REGION_MB_RTU, 4u, 4321u, GW_Q_GOOD);
    }
    GwImage_SetQuality(GW_REGION_MB_RTU, 4u, GW_Q_COMM_FAIL);

    /* slot 5: never written at all - stays UNKNOWN for the whole run */

    /* LOCAL_IO used to get a slow counter here so that a second region was
     * demonstrably alive. gw_localio.c owns that region now and writes it from
     * real pins, so the counter is gone: two writers on one slot is the one
     * thing the ownership rule in gw_image.h forbids outright, and the symptom
     * would be a relay state that flickers between what the pin is doing and
     * whatever the animator last stored. */
}
#endif

void GwImage_Step(void) {
    static uint32_t loopCount;
    static uint32_t nextAgeMs;
    uint32_t now;
    uint8_t i;

    if (!gReady) return;

    now = HAL_GetTick();
    loopCount++;

    GwImage_SysSetU32(GW_SYS_OFF_UPTIME_MS, now);
    GwImage_SysSetU32(GW_SYS_OFF_LOOP_COUNT, loopCount);

#if GW_IMAGE_DEMO
    demoStep();
#endif

    /* Ageing once a second is plenty - staleness is measured in seconds - and
     * keeps the superloop cost of this function at roughly nothing. */
    if ((int32_t)(now - nextAgeMs) < 0) return;
    nextAgeMs = now + 1000u;

    for (i = 0u; i < gRegionCount; i++) {
        const GwRegionDesc* d = &gDesc[i];
        uint8_t* qual;
        const uint32_t* stamps;
        uint16_t s;

        if (d->slots == 0u) continue;

        qual = &gImage[gBase[i] + d->qualOff];
        stamps = (const uint32_t*)(void*)&gImage[gBase[i] + d->stampOff];

        for (s = 0u; s < d->slots; s++) {
            /* Only GOOD decays. COMM_FAIL and the rest are driver verdicts and
             * are not this function's to overwrite; UNKNOWN means the slot has
             * never been written and ageing it would say nothing new. */
            if (qual[s] != GW_Q_GOOD) continue;
            if ((uint32_t)(now - stamps[s]) > GW_DEFAULT_STALE_MS) {
                qual[s] = GW_Q_STALE;
            }
        }
    }
}

/* ==========================================================================
 * Layout enquiry
 * ==========================================================================
 */

uint8_t GwImage_RegionCount(void) { return gReady ? gRegionCount : 0u; }

const GwRegionDesc* GwImage_Desc(uint8_t region) {
    if (!gReady) return (const GwRegionDesc*)0;
    return descOf(region);
}

const GwRegionDesc* GwImage_DescAt(uint8_t index) {
    if (!gReady || index >= gRegionCount) return (const GwRegionDesc*)0;
    return &gDesc[index];
}

uint16_t GwImage_TotalBytes(void) { return gTotalBytes; }

uint32_t GwImage_ConfigCrc32(void) { return GW_CONFIG_CRC32; }

uint16_t GwImage_ConfigVersion(void) { return GW_CONFIG_VERSION; }

/* ==========================================================================
 * Raw access
 * ==========================================================================
 */

uint8_t GwImage_Read(uint8_t region, uint16_t off, uint16_t len, uint8_t* dst) {
    const GwRegionDesc* d;

    if (!gReady) return GW_ST_BUSY;
    d = descOf(region);
    if (d == 0) return GW_ST_BAD_REGION;

    /* Both halves of the range check matter. off alone can be past the end, and
     * off+len can wrap - which is the shape of the overflow that v1's unchecked
     * rxIndex allowed. The addition is done in 32 bits so it cannot wrap. */
    if ((uint32_t)off + (uint32_t)len > (uint32_t)d->byteLen) return GW_ST_RANGE;
    if (len == 0u) return GW_ST_OK;

    memcpy(dst, baseOf(region) + off, len);
    return GW_ST_OK;
}

uint8_t GwImage_Write(uint8_t region, uint16_t off, const uint8_t* src, uint16_t len) {
    const GwRegionDesc* d;

    if (!gReady) return GW_ST_BUSY;
    d = descOf(region);
    if (d == 0) return GW_ST_BAD_REGION;
    if ((d->flags & GW_REGF_WRITABLE) == 0u) return GW_ST_NOT_WRITABLE;
    if ((uint32_t)off + (uint32_t)len > (uint32_t)d->byteLen) return GW_ST_RANGE;
    if (len == 0u) return GW_ST_OK;

    memcpy(baseOf(region) + off, src, len);
    return GW_ST_OK;
}

/* ==========================================================================
 * Slot access
 * ==========================================================================
 */

uint8_t GwImage_SetU32(uint8_t region, uint16_t slot, uint32_t value, uint8_t quality) {
    uint32_t* v;
    uint32_t* t;
    uint8_t* q;
    uint8_t st = slotPtrs(region, slot, &v, &t, &q);
    if (st != GW_ST_OK) return st;

    /* Order matters for a lock-free reader: value first, then the timestamp
     * that vouches for it, then the quality that unlocks it. A reader that
     * interleaves sees an older-but-consistent tuple, never a new value stamped
     * with an old time. */
    *v = value;
    *t = HAL_GetTick();
    *q = quality;
    return GW_ST_OK;
}

uint8_t GwImage_SetF32(uint8_t region, uint16_t slot, float value, uint8_t quality) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits)); /* not a cast - no aliasing games */
    return GwImage_SetU32(region, slot, bits, quality);
}

uint8_t GwImage_SetQuality(uint8_t region, uint16_t slot, uint8_t quality) {
    uint8_t* q;
    uint8_t st = slotPtrs(region, slot, (uint32_t**)0, (uint32_t**)0, &q);
    if (st != GW_ST_OK) return st;
    *q = quality;
    return GW_ST_OK;
}

uint8_t GwImage_GetSlot(uint8_t region, uint16_t slot, uint32_t* value, uint32_t* stampMs,
                        uint8_t* quality) {
    uint32_t* v;
    uint32_t* t;
    uint8_t* q;
    uint8_t st = slotPtrs(region, slot, &v, &t, &q);
    if (st != GW_ST_OK) return st;

    if (value != 0) *value = *v;
    if (stampMs != 0) *stampMs = *t;
    if (quality != 0) *quality = *q;
    return GW_ST_OK;
}

/* ==========================================================================
 * SYS helpers
 * ==========================================================================
 * Offsets are compile-time constants from gw_model.h, but they are still range
 * checked: a typo in an offset should produce nothing rather than corrupt the
 * neighbouring region.
 */

void GwImage_SysSetU32(uint16_t off, uint32_t value) {
    uint8_t* base;
    if (!gReady || (uint32_t)off + 4u > GW_SYS_SIZE) return;
    base = baseOf(GW_REGION_SYS);
    if (base == 0) return;
    gwWr32(base + off, value);
}

void GwImage_SysSetU16(uint16_t off, uint16_t value) {
    uint8_t* base;
    if (!gReady || (uint32_t)off + 2u > GW_SYS_SIZE) return;
    base = baseOf(GW_REGION_SYS);
    if (base == 0) return;
    gwWr16(base + off, value);
}

void GwImage_SysSetU8(uint16_t off, uint8_t value) {
    uint8_t* base;
    if (!gReady || off >= GW_SYS_SIZE) return;
    base = baseOf(GW_REGION_SYS);
    if (base == 0) return;
    base[off] = value;
}

uint32_t GwImage_SysGetU32(uint16_t off) {
    const uint8_t* base;
    if (!gReady || (uint32_t)off + 4u > GW_SYS_SIZE) return 0u;
    base = baseOf(GW_REGION_SYS);
    if (base == 0) return 0u;
    return gwRd32(base + off);
}

void GwImage_SetFault(uint32_t bits) {
    GwImage_SysSetU32(GW_SYS_OFF_FAULT_BITS, GwImage_SysGetU32(GW_SYS_OFF_FAULT_BITS) | bits);
}

void GwImage_ClearFault(uint32_t bits) {
    GwImage_SysSetU32(GW_SYS_OFF_FAULT_BITS, GwImage_SysGetU32(GW_SYS_OFF_FAULT_BITS) & ~bits);
}
