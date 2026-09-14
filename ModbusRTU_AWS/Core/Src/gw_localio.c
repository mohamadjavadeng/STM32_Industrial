/**
 * gw_localio.c - PD0..PD3 relays, PD4..PD7 pulled-up inputs.
 *
 * See gw_localio.h for the contract and gw_link_cfg.h for the tunables. The
 * only two things in here that are not obvious are the debounce and the
 * lost-link failsafe, and both are commented where they happen.
 */
#include "gw_localio.h"

#include "gw_image.h"
#include "main.h"

/* ==========================================================================
 * Pin tables
 * ==========================================================================
 * Index order is terminal order: kDoPin[0] is the terminal silkscreened
 * "RELAY 1", which is slot GW_LIO_DO_BASE, which is bit 0 of every mask in
 * this file and of the DO word slot. One ordering, used everywhere, so there
 * is never a place where relay 1 has to be translated into something else.
 */
static GPIO_TypeDef* const kIoPort = GPIOD;

static const uint16_t kDoPin[GW_LIO_DO_COUNT] = {
    GPIO_PIN_0, /* relay 1 */
    GPIO_PIN_1, /* relay 2 */
    GPIO_PIN_2, /* relay 3 */
    GPIO_PIN_3, /* relay 4 */
};

static const uint16_t kDiPin[GW_LIO_DI_COUNT] = {
    GPIO_PIN_4, /* input 1 */
    GPIO_PIN_5, /* input 2 */
    GPIO_PIN_6, /* input 3 */
    GPIO_PIN_7, /* input 4 */
};

/* ==========================================================================
 * State
 * ==========================================================================
 */
static bool gEnabled;      /* false when the image cannot hold the slot map   */
static uint8_t gCmdMask;   /* what the cloud has asked for                    */
static uint8_t gLiveMask;  /* what the pins are actually doing                */
static uint8_t gRawMask;   /* inputs as sampled, before debounce              */
static uint8_t gInputMask; /* inputs after debounce - what we publish         */
static uint32_t gBitSettledMs[GW_LIO_DI_COUNT]; /* per-bit settling window    */
static uint32_t gNextScanMs;

/* Lost-link failsafe bookkeeping. See stepFailsafe(). */
static uint32_t gLastLinkRx;
static uint32_t gLastLinkRxMs;
static bool gFailsafeLatched;

/* ==========================================================================
 * Pin helpers - the only place polarity is applied
 * ==========================================================================
 */

/** Drives one relay from its LOGICAL state (1 = energized). */
static void driveRelay(uint8_t index, bool on) {
#if GW_LIO_DO_ACTIVE_LOW
    GPIO_PinState level = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
#else
    GPIO_PinState level = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
    HAL_GPIO_WritePin(kIoPort, kDoPin[index], level);
}

/** Reads one input as its LOGICAL state (1 = contact closed / signal present). */
static bool readInput(uint8_t index) {
    GPIO_PinState level = HAL_GPIO_ReadPin(kIoPort, kDiPin[index]);
#if GW_LIO_DI_ACTIVE_LOW
    return level == GPIO_PIN_RESET;
#else
    return level == GPIO_PIN_SET;
#endif
}

/* ==========================================================================
 * Image publishing
 * ==========================================================================
 * Every slot is rewritten on every scan, not only when a bit changes.
 *
 * That is deliberate. GwImage_Step() demotes a slot GOOD -> STALE once its
 * timestamp is older than GW_DEFAULT_STALE_MS, and an input that has correctly
 * read "open" for an hour is not stale - it is fresh and unchanged. Publishing
 * only on change would mark every quiet channel untrustworthy after fifteen
 * seconds, which is precisely backwards. Ten stores per scan is nothing.
 */
static void publish(void) {
    uint8_t i;

    for (i = 0u; i < GW_LIO_DO_COUNT; ++i) {
        GwImage_SetU32(GW_REGION_LOCAL_IO, (uint16_t)(GW_LIO_DO_BASE + i),
                       (uint32_t)((gLiveMask >> i) & 1u), GW_Q_GOOD);
    }
    for (i = 0u; i < GW_LIO_DI_COUNT; ++i) {
        GwImage_SetU32(GW_REGION_LOCAL_IO, (uint16_t)(GW_LIO_DI_BASE + i),
                       (uint32_t)((gInputMask >> i) & 1u), GW_Q_GOOD);
    }
    GwImage_SetU32(GW_REGION_LOCAL_IO, GW_LIO_DO_WORD, gLiveMask, GW_Q_GOOD);
    GwImage_SetU32(GW_REGION_LOCAL_IO, GW_LIO_DI_WORD, gInputMask, GW_Q_GOOD);
}

/* ==========================================================================
 * Debounce
 * ==========================================================================
 * A bit has to hold a new reading for GW_LIO_DEBOUNCE_MS before it is believed.
 * Mechanical contacts ring for a few milliseconds on every transition, and
 * without this the gateway would publish a burst of edges to the cloud for one
 * physical event - which looks to an operator exactly like a fault they do not
 * have.
 *
 * Per-bit timers rather than one shared timer: two inputs changing in the same
 * millisecond must not extend each other's settling window.
 */
static void stepInputs(uint32_t now) {
    uint8_t i;
    uint8_t fresh = 0u;

    for (i = 0u; i < GW_LIO_DI_COUNT; ++i) {
        if (readInput(i)) fresh |= (uint8_t)(1u << i);
    }

    for (i = 0u; i < GW_LIO_DI_COUNT; ++i) {
        uint8_t bit = (uint8_t)(1u << i);
        bool nowRaw = (fresh & bit) != 0u;
        bool wasRaw = (gRawMask & bit) != 0u;
        bool settled = (gInputMask & bit) != 0u;

        if (nowRaw != wasRaw) {
            /* Still bouncing - restart this bit's window. */
            gBitSettledMs[i] = now;
        } else if (nowRaw != settled &&
                   (uint32_t)(now - gBitSettledMs[i]) >= GW_LIO_DEBOUNCE_MS) {
            if (nowRaw) {
                gInputMask |= bit;
            } else {
                gInputMask &= (uint8_t)~bit;
            }
        }
    }

    gRawMask = fresh;
}

/* ==========================================================================
 * Lost-link failsafe
 * ==========================================================================
 * Keyed on the link's accepted-frame counter in the SYS region, not on the time
 * of the last relay command. A relay switched on and then legitimately left
 * alone for an hour must not drop; a gateway whose ESP32 has stopped talking to
 * it at all is a different situation, and that is what this detects.
 *
 * Off by default (GW_LIO_FAILSAFE_MS 0), because whether outputs should drop or
 * hold on lost comms is a property of the machine, not of the firmware: a
 * conveyor should stop, a heater holding a setpoint probably should not.
 * Whoever commissions the panel has to make that call deliberately.
 */
static void stepFailsafe(uint32_t now) {
#if GW_LIO_FAILSAFE_MS > 0
    uint32_t rx = GwImage_SysGetU32(GW_SYS_OFF_LINK_RX);

    if (rx != gLastLinkRx) {
        gLastLinkRx = rx;
        gLastLinkRxMs = now;
        if (gFailsafeLatched) {
            gFailsafeLatched = false;
            GwImage_ClearFault(GW_FAULT_EXT_REGION_STALE);
        }
        return;
    }

    if (!gFailsafeLatched && (uint32_t)(now - gLastLinkRxMs) >= GW_LIO_FAILSAFE_MS) {
        gFailsafeLatched = true;
        GwLocalIO_Failsafe();
        GwImage_SetFault(GW_FAULT_EXT_REGION_STALE);
    }
#else
    (void)now;
    (void)gLastLinkRx;
    (void)gLastLinkRxMs;
#endif
}

/* ==========================================================================
 * Public API
 * ==========================================================================
 */

bool GwLocalIO_Init(void) {
    const GwRegionDesc* desc;
    uint8_t i;

    gEnabled = false;
    gCmdMask = 0u;
    gLiveMask = 0u;
    gRawMask = 0u;
    gInputMask = 0u;
    gFailsafeLatched = false;

    /* Refuse rather than write past the region. A LOCAL_IO sized smaller than
     * the slot map is a configuration mistake in gw_link_cfg.h, and serving
     * eight plausible zeroes out of the wrong memory would hide it. */
    desc = GwImage_Desc(GW_REGION_LOCAL_IO);
    if (desc == NULL) return false;
    if ((desc->flags & GW_REGF_ENABLED) == 0u) return false;
    if (desc->slots < GW_LIO_SLOT_MAX) return false;

    /* The pins themselves are configured by MX_GPIO_Init(), generated from
     * ModbusRTU_AWS.ioc: PD0..PD3 GPIO_Output with PinState GPIO_PIN_SET, and
     * PD4..PD7 GPIO_Input with GPIO_PULLUP. The .ioc is the source of truth for
     * peripheral init in this project, so a driver that configured its own pins
     * would be a second, invisible answer to the same question - and the one
     * CubeMX does not know about. This module only drives and samples.
     *
     * That makes the call order in main() load-bearing: MX_GPIO_Init() first,
     * this second. It already is.
     *
     * Drive the relays to their de-energized level anyway. The .ioc PinState
     * has already done it for the polarity the .ioc was written for; repeating
     * it here in logical terms means GW_LIO_DO_ACTIVE_LOW always has the final
     * say, within microseconds of boot, even if the two ever disagree. */
    for (i = 0u; i < GW_LIO_DO_COUNT; ++i) {
        driveRelay(i, false);
    }

    /* Seed the debounced state from one sample instead of letting every input
     * spend its first GW_LIO_DEBOUNCE_MS reported as open. A contact that is
     * already closed at power-up is a real state, not a transition. */
    gInputMask = 0u;
    for (i = 0u; i < GW_LIO_DI_COUNT; ++i) {
        if (readInput(i)) gInputMask |= (uint8_t)(1u << i);
        gBitSettledMs[i] = HAL_GetTick();
    }
    gRawMask = gInputMask;

    gLastLinkRx = 0u;
    gLastLinkRxMs = HAL_GetTick();
    gNextScanMs = HAL_GetTick();
    gEnabled = true;

    publish();
    return true;
}

void GwLocalIO_Step(void) {
    uint32_t now;

    if (!gEnabled) return;

    now = HAL_GetTick();
    if ((int32_t)(now - gNextScanMs) < 0) return;
    gNextScanMs = now + GW_LIO_SCAN_MS;

    stepFailsafe(now);

    if (gCmdMask != gLiveMask) {
        uint8_t i;
        for (i = 0u; i < GW_LIO_DO_COUNT; ++i) {
            uint8_t bit = (uint8_t)(1u << i);
            if (((gCmdMask ^ gLiveMask) & bit) != 0u) {
                driveRelay(i, (gCmdMask & bit) != 0u);
            }
        }
        gLiveMask = gCmdMask;
    }

    stepInputs(now);
    publish();
}

uint8_t GwLocalIO_WriteSlot(uint16_t slot, uint32_t value, uint8_t encoding) {
    if (!gEnabled) return GW_ST_BAD_REGION;

    /* A relay is a bit. A float that happens to be 0.7 has no correct answer
     * here, and rounding it would be a guess at what the operator meant. */
    if (encoding != GW_ENC_U32 && encoding != GW_ENC_I32) return GW_ST_BAD_LEN;

    if (slot >= GW_LIO_DO_BASE && slot < (GW_LIO_DO_BASE + GW_LIO_DO_COUNT)) {
        uint8_t bit = (uint8_t)(1u << (slot - GW_LIO_DO_BASE));
        if (value > 1u) return GW_ST_RANGE;
        if (value != 0u) {
            gCmdMask |= bit;
        } else {
            gCmdMask &= (uint8_t)~bit;
        }
        gFailsafeLatched = false;
        return GW_ST_OK;
    }

    if (slot == GW_LIO_DO_WORD) {
        if (value > 0x0Fu) return GW_ST_RANGE;
        gCmdMask = (uint8_t)value;
        gFailsafeLatched = false;
        return GW_ST_OK;
    }

    /* Inputs, and the DI word, are what this driver reports. Writing one would
     * be writing to a terminal that is physically an input. */
    if ((slot >= GW_LIO_DI_BASE && slot < (GW_LIO_DI_BASE + GW_LIO_DI_COUNT)) ||
        slot == GW_LIO_DI_WORD) {
        return GW_ST_NOT_WRITABLE;
    }

    return GW_ST_RANGE;
}

uint8_t GwLocalIO_Relays(void) { return gLiveMask; }

uint8_t GwLocalIO_Inputs(void) { return gInputMask; }

void GwLocalIO_Failsafe(void) {
    uint8_t i;

    gCmdMask = 0u;
    if (!gEnabled) return;

    for (i = 0u; i < GW_LIO_DO_COUNT; ++i) {
        driveRelay(i, false);
    }
    gLiveMask = 0u;
    publish();
}
