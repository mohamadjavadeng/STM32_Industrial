/**
 * linkv2_test_main.cpp - bring-up console for the ESP32 <-> STM32 link.
 * ---------------------------------------------------------------------------
 *
 * A standalone firmware whose only job is to prove the link. No WiFi, no TLS,
 * no certificates, no cloud - so it can be run on a bare bench with two boards
 * and a USB cable, and a failure here can only be the link.
 *
 *   pio run -e linkv2 -t upload -t monitor
 *
 * Then press keys in the serial monitor. `a` runs everything and prints a
 * pass/fail summary; `?` lists the menu.
 *
 * The step-by-step procedure, the expected byte traces and what each failure
 * means are in docs/LINK_V2_TESTING.html. This file is the instrument; that
 * document is the method.
 *
 * WHY A CONSOLE AND NOT A UNIT TEST
 *   Everything worth testing here involves two chips, a cable and a DMA
 *   controller. A host-side unit test would exercise the parser, which is the
 *   part least likely to be wrong; it cannot exercise a ground loop, a baud
 *   mismatch, an overrun, or a reply that arrives after its timeout. Those are
 *   what actually break, and they need real hardware.
 *
 * This file is excluded from the gateway builds by build_src_filter in
 * platformio.ini - see the comments there.
 */
#include <Arduino.h>
#include <string.h>

#include "crc16_modbus.h"
#include "gw_model.h"
#include "stm_link_v2.h"

// Running tally for the automated sequence.
static uint32_t gPass;
static uint32_t gFail;

/* ==========================================================================
 * Small reporting helpers
 * ==========================================================================
 */

static void banner(const char* title) {
    Serial.println();
    Serial.printf("=== %s %.*s\r\n", title, (int)(60 - strlen(title)),
                  "==========================================================");
}

/** Records one check. Every test reports through here so the tally is honest. */
static bool check(bool ok, const char* what, const char* detail = "") {
    if (ok) {
        gPass++;
        Serial.printf("  [ OK ] %s %s\r\n", what, detail);
    } else {
        gFail++;
        Serial.printf("  [FAIL] %s %s\r\n", what, detail);
    }
    return ok;
}

/** Formats a transport result and, when relevant, the device status behind it. */
static const char* resultText(StmV2Status st) {
    static char buf[48];
    if (st == V2_ERR_DEVICE) {
        snprintf(buf, sizeof(buf), "DEVICE/%s", gwStatusName(stmLinkV2.deviceStatus()));
    } else {
        snprintf(buf, sizeof(buf), "%s", stmV2StatusName(st));
    }
    return buf;
}

/* ==========================================================================
 * T1 - ECHO
 * ==========================================================================
 * The foundation test. It touches the wiring, the baud rate, the DMA on both
 * sides, the framing and the CRC - and nothing else. Until this passes at every
 * size, no failure further up means anything.
 *
 * The sizes are chosen deliberately:
 *   0     an empty payload - the degenerate frame, and the one an inference
 *         based parser gets wrong
 *   1     the minimum real payload
 *   64    one byte more than the whole receive buffer of the old v1 firmware
 *   255   crosses the 8-bit length boundary that v1's single-byte count had
 *   1024  the maximum payload, which is also the largest DMA transfer
 */
static void testEcho() {
    banner("T1  ECHO - transport, framing, CRC");

    static const uint16_t sizes[] = {0, 1, 64, 255, 1024};

    // static, not automatic: two kilobytes of buffers on the Arduino loop task
    // stack is not worth the risk when the file is single-threaded anyway.
    static uint8_t tx[GW_MAX_PAYLOAD];
    static uint8_t rx[GW_MAX_PAYLOAD];

    for (uint16_t i = 0; i < GW_MAX_PAYLOAD; i++) tx[i] = (uint8_t)(i * 7 + 13);

    for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
        const uint16_t n = sizes[i];
        uint16_t got = 0;
        char detail[80];

        StmV2Status st = stmLinkV2.echo(tx, n, rx, (uint16_t)sizeof(rx), got);

        bool ok = (st == V2_OK) && (got == n) && (n == 0 || memcmp(tx, rx, n) == 0);
        snprintf(detail, sizeof(detail), "%4u bytes -> %s, %u back, %lu us", n, resultText(st), got,
                 (unsigned long)stmLinkV2.stats().lastRttUs);
        check(ok, "echo", detail);
    }
}

/* ==========================================================================
 * T2 - SYNC
 * ==========================================================================
 */
static void testSync() {
    banner("T2  SYNC - identity and uptime");

    GwSyncRsp a{};
    StmV2Status st = stmLinkV2.sync(a);
    if (!check(st == V2_OK, "sync", resultText(st))) return;

    Serial.printf("       STM32 firmware   %u.%u.%u\r\n", a.fwMajor, a.fwMinor, a.fwPatch);
    Serial.printf("       protocol         v%u.%u\r\n", a.protoVersion >> 4,
                  a.protoVersion & 0x0F);
    Serial.printf("       uptime           %lu ms\r\n", (unsigned long)a.stmUptimeMs);
    Serial.printf("       config           CRC %08lX  version %u\r\n",
                  (unsigned long)a.configCrc32, a.configVersion);
    Serial.printf("       capabilities     0x%04X\r\n", a.capabilities);

    check((a.protoVersion >> 4) == GW_PROTO_VERSION_MAJOR, "protocol major matches");

    // Uptime must advance. A frozen value means the SYNC reply is cached
    // somewhere, or the STM32 is stuck with its SysTick dead - both of which
    // would otherwise look like a perfectly healthy link.
    delay(250);
    GwSyncRsp b{};
    st = stmLinkV2.sync(b);
    if (!check(st == V2_OK, "sync again", resultText(st))) return;

    char d[64];
    snprintf(d, sizeof(d), "+%ld ms over 250 ms", (long)(b.stmUptimeMs - a.stmUptimeMs));
    check(b.stmUptimeMs > a.stmUptimeMs, "uptime advances", d);
}

/* ==========================================================================
 * T3 - RD_META
 * ==========================================================================
 */
static GwImageMap gMap;  // kept for the later tests, which need the layout

static void testMeta() {
    banner("T3  RD_META - image layout");

    StmV2Status st = stmLinkV2.readMeta(gMap);
    if (!check(st == V2_OK, "read meta", resultText(st))) return;

    Serial.printf("       image %u bytes, %u regions, config CRC %08lX v%u\r\n", gMap.imageBytes,
                  gMap.regionCount, (unsigned long)gMap.configCrc32, gMap.configVersion);
    Serial.println("       id  name       flags  slots  bytes   value  stamp   qual");
    for (uint8_t i = 0; i < gMap.regionCount; i++) {
        const GwRegionDesc& d = gMap.region[i];
        Serial.printf("       %02X  %-9s  0x%02X  %5u  %5u  %5u  %5u  %5u\r\n", d.region,
                      gwRegionName(d.region), d.flags, d.slots, d.byteLen, d.valueOff, d.stampOff,
                      d.qualOff);
    }

    check(gMap.regionCount > 0, "at least one region");
    check(gMap.find(GW_REGION_SYS) != nullptr, "SYS region present");
    check(gMap.find(GW_REGION_MB_RTU) != nullptr, "MB_RTU region present");

    // The layout arithmetic the ESP32 will rely on for every read from now on.
    // Getting this wrong shifts every tag by a few bytes, which produces
    // plausible nonsense rather than an error.
    const GwRegionDesc* rtu = gMap.find(GW_REGION_MB_RTU);
    if (rtu != nullptr) {
        bool consistent = (rtu->valueOff == 0) &&
                          (rtu->stampOff == rtu->slots * GW_SLOT_VALUE_BYTES) &&
                          (rtu->qualOff == rtu->slots * (GW_SLOT_VALUE_BYTES + GW_SLOT_STAMP_BYTES)) &&
                          (rtu->byteLen == rtu->slots * GW_SLOT_TOTAL_BYTES);
        check(consistent, "MB_RTU block offsets are self-consistent");
    }
}

/* ==========================================================================
 * T4 - SYS region
 * ==========================================================================
 */
static void testSysRegion() {
    banner("T4  RD_REGION on SYS - diagnostics");

    uint8_t sys[GW_SYS_SIZE];
    StmV2Status st = stmLinkV2.readRegion(GW_REGION_SYS, 0, GW_SYS_SIZE, sys);
    if (!check(st == V2_OK, "read SYS", resultText(st))) return;

    Serial.printf("       uptime       %lu ms\r\n", (unsigned long)gwRd32(&sys[GW_SYS_OFF_UPTIME_MS]));
    Serial.printf("       firmware     %u.%u.%u, protocol v%u.%u\r\n", sys[GW_SYS_OFF_FW_MAJOR],
                  sys[GW_SYS_OFF_FW_MINOR], sys[GW_SYS_OFF_FW_PATCH],
                  sys[GW_SYS_OFF_PROTO_VER] >> 4, sys[GW_SYS_OFF_PROTO_VER] & 0x0F);
    Serial.printf("       link frames  rx %lu  tx %lu\r\n",
                  (unsigned long)gwRd32(&sys[GW_SYS_OFF_LINK_RX]),
                  (unsigned long)gwRd32(&sys[GW_SYS_OFF_LINK_TX]));
    Serial.printf("       link errors  crc %lu  frame %lu  dropped %lu\r\n",
                  (unsigned long)gwRd32(&sys[GW_SYS_OFF_LINK_CRC_ERR]),
                  (unsigned long)gwRd32(&sys[GW_SYS_OFF_LINK_FRM_ERR]),
                  (unsigned long)gwRd32(&sys[GW_SYS_OFF_LINK_DROPPED]));
    Serial.printf("       loop count   %lu\r\n", (unsigned long)gwRd32(&sys[GW_SYS_OFF_LOOP_COUNT]));
    Serial.printf("       fault bits   0x%08lX\r\n",
                  (unsigned long)gwRd32(&sys[GW_SYS_OFF_FAULT_BITS]));

    check(gwRd32(&sys[GW_SYS_OFF_LINK_RX]) > 0, "STM32 counted our requests");
    check(gwRd32(&sys[GW_SYS_OFF_LOOP_COUNT]) > 0, "STM32 superloop is running");

    // A fault bit here is not a link failure, so it is a warning rather than a
    // failed check - but it is the first thing to look at when throughput is
    // disappointing.
    uint32_t faults = gwRd32(&sys[GW_SYS_OFF_FAULT_BITS]);
    if (faults & GW_FAULT_LINK_OVERRUN) {
        Serial.println("       NOTE: GW_FAULT_LINK_OVERRUN set - the STM32 superloop stalled long "
                       "enough to lose bytes at least once.");
    }
}

/* ==========================================================================
 * T5 - slots and quality
 * ==========================================================================
 */
static void printSlots(uint8_t region, uint16_t first, uint16_t count) {
    GwSlot slots[16];
    if (count > 16) count = 16;

    StmV2Status st = stmLinkV2.readSlots(gMap, region, first, count, slots);
    if (st != V2_OK) {
        Serial.printf("       read failed: %s\r\n", resultText(st));
        return;
    }

    Serial.println("       slot       raw        as float     stamp(ms)   quality");
    for (uint16_t i = 0; i < count; i++) {
        Serial.printf("       %4u  %10lu  %11.4f  %10lu   %s\r\n", first + i,
                      (unsigned long)slots[i].raw, StmLinkV2::slotAsFloat(slots[i].raw),
                      (unsigned long)slots[i].stampMs, gwQualityName(slots[i].quality));
    }
}

static void testSlots() {
    banner("T5  Slots - value, timestamp and quality together");

    if (!gMap.valid()) {
        check(false, "need RD_META first");
        return;
    }

    Serial.println("       MB_RTU, the region the demo generator fills:");
    printSlots(GW_REGION_MB_RTU, 0, 8);

    // The demo animation in gw_image.c defines what these slots mean. Checking
    // them proves the whole path end to end: an STM32 driver wrote a slot, the
    // image stored it with a stamp and a quality, and the bytes arrived here
    // meaning the same thing.
    GwSlot s[8];
    StmV2Status st = stmLinkV2.readSlots(gMap, GW_REGION_MB_RTU, 0, 8, s);
    if (!check(st == V2_OK, "read MB_RTU slots 0..7", resultText(st))) return;

    check(s[0].quality == GW_Q_GOOD, "slot 0 (counter) is GOOD");
    check(s[4].quality == GW_Q_COMM_FAIL, "slot 4 is COMM_FAIL as designed");
    check(s[5].quality == GW_Q_UNKNOWN, "slot 5 never written -> UNKNOWN");

    // The counter must move between two reads. A frozen value with GOOD quality
    // would be exactly the failure link v1 could not report - data that looks
    // fine and is not being refreshed.
    uint32_t before = s[0].raw;
    delay(500);
    st = stmLinkV2.readSlots(gMap, GW_REGION_MB_RTU, 0, 1, s);
    if (!check(st == V2_OK, "re-read slot 0", resultText(st))) return;

    char d[64];
    snprintf(d, sizeof(d), "%lu -> %lu over 500 ms", (unsigned long)before, (unsigned long)s[0].raw);
    check(s[0].raw != before, "the image is being refreshed", d);
}

/* ==========================================================================
 * T6 - WR_REGION
 * ==========================================================================
 */
static void testWrite() {
    banner("T6  WR_REGION - the externally owned region");

    if (!gMap.valid()) {
        check(false, "need RD_META first");
        return;
    }

    const GwRegionDesc* tcp = gMap.find(GW_REGION_MB_TCP);
    if (tcp == nullptr || (tcp->flags & GW_REGF_WRITABLE) == 0) {
        check(false, "MB_TCP is not writable on this firmware");
        return;
    }

    // Write value + stamp + quality for slot 0 the way a Modbus TCP master
    // running on this chip eventually will.
    const uint16_t slot = 0;
    const uint32_t value = (uint32_t)millis();
    uint8_t buf[4];

    gwWr32(buf, value);
    StmV2Status st = stmLinkV2.writeRegion(
        GW_REGION_MB_TCP, (uint16_t)(tcp->valueOff + slot * GW_SLOT_VALUE_BYTES), buf, 4);
    if (!check(st == V2_OK, "write value", resultText(st))) return;

    uint8_t q = GW_Q_GOOD;
    st = stmLinkV2.writeRegion(GW_REGION_MB_TCP, (uint16_t)(tcp->qualOff + slot), &q, 1);
    check(st == V2_OK, "write quality", resultText(st));

    uint8_t back[4];
    st = stmLinkV2.readRegion(GW_REGION_MB_TCP,
                              (uint16_t)(tcp->valueOff + slot * GW_SLOT_VALUE_BYTES), 4, back);
    if (!check(st == V2_OK, "read it back", resultText(st))) return;

    char d[48];
    snprintf(d, sizeof(d), "wrote %lu, read %lu", (unsigned long)value, (unsigned long)gwRd32(back));
    check(gwRd32(back) == value, "value survived the round trip", d);
}

/* ==========================================================================
 * T7 - error handling
 * ==========================================================================
 * Error paths that have never been exercised are decoration. Each of these is a
 * defect the old protocol actually had, now checked rather than assumed.
 */
static void testNegative() {
    banner("T7  Error handling - the paths nobody exercises");

    uint8_t frame[32];
    uint8_t out[64];
    uint16_t got = 0;
    StmV2Status st;

    // --- corrupted CRC ----------------------------------------------------
    // v1 answered a bad CRC with BB 01 00, which was indistinguishable from a
    // failed field read. Here it is a distinct status.
    {
        frame[GW_REQ_OFF_SOF] = GW_SOF_REQ;
        frame[GW_REQ_OFF_SEQ] = 0x41;
        frame[GW_REQ_OFF_OP] = GW_OP_SYNC;
        frame[GW_REQ_OFF_REG] = 0;
        gwWr16(&frame[GW_REQ_OFF_OFFSET], 0);
        gwWr16(&frame[GW_REQ_OFF_LEN], 0);
        size_t n = crc16ModbusAppend(frame, GW_REQ_HEADER_LEN);
        frame[n - 1] ^= 0xFF;  // break the CRC's high byte

        st = stmLinkV2.sendRaw(frame, (uint16_t)n, 0x41, out, sizeof(out), got);
        check(st == V2_ERR_DEVICE && stmLinkV2.deviceStatus() == GW_ST_BAD_CRC, "bad CRC rejected",
              resultText(st));
    }

    // --- unknown opcode ----------------------------------------------------
    {
        uint16_t dummy = 0;
        st = stmLinkV2.transact(0x55, 0, 0, nullptr, 0, out, sizeof(out), dummy);
        check(st == V2_ERR_DEVICE && stmLinkV2.deviceStatus() == GW_ST_BAD_CMD,
              "unknown opcode -> BAD_CMD", resultText(st));
    }

    // --- unknown region ----------------------------------------------------
    {
        uint8_t buf[4];
        st = stmLinkV2.readRegion(0x7E, 0, 4, buf);
        check(st == V2_ERR_DEVICE && stmLinkV2.deviceStatus() == GW_ST_BAD_REGION,
              "unknown region -> BAD_REGION", resultText(st));
    }

    // --- past the end of a region -----------------------------------------
    // The check that turns a hostile length into an error instead of a read of
    // whatever follows the region in RAM.
    {
        uint8_t buf[GW_SYS_SIZE * 2];
        st = stmLinkV2.readRegion(GW_REGION_SYS, 0, GW_SYS_SIZE * 2, buf);
        check(st == V2_ERR_DEVICE && stmLinkV2.deviceStatus() == GW_ST_RANGE,
              "over-long read -> RANGE", resultText(st));
    }

    // --- impossible declared length ---------------------------------------
    // The direct descendant of v1 quirk Q8: a frame claiming more payload than
    // the buffer holds. On v1 that was a live overflow; here it must be refused
    // from the header alone, before a single payload byte is stored.
    {
        frame[GW_REQ_OFF_SOF] = GW_SOF_REQ;
        frame[GW_REQ_OFF_SEQ] = 0x42;
        frame[GW_REQ_OFF_OP] = GW_OP_ECHO;
        frame[GW_REQ_OFF_REG] = 0;
        gwWr16(&frame[GW_REQ_OFF_OFFSET], 0);
        gwWr16(&frame[GW_REQ_OFF_LEN], 0x7FFF);
        size_t n = crc16ModbusAppend(frame, GW_REQ_HEADER_LEN);

        st = stmLinkV2.sendRaw(frame, (uint16_t)n, 0x42, out, sizeof(out), got);
        check(st == V2_ERR_DEVICE && stmLinkV2.deviceStatus() == GW_ST_BAD_LEN,
              "impossible length -> BAD_LEN", resultText(st));
    }

    // --- writing a region we do not own ------------------------------------
    {
        uint8_t v[4] = {1, 2, 3, 4};
        st = stmLinkV2.writeRegion(GW_REGION_MB_RTU, 0, v, 4);
        check(st == V2_ERR_DEVICE && stmLinkV2.deviceStatus() == GW_ST_NOT_WRITABLE,
              "write to a driver-owned region -> NOT_WRITABLE", resultText(st));
    }

    // --- duplicate sequence number ----------------------------------------
    // Two different ECHO requests sent with the same SEQ. The STM32 must answer
    // the second from its reply cache without executing it, so the payload that
    // comes back is the FIRST one. That is what makes a retried write safe.
    {
        uint8_t a = 'A';
        uint8_t b = 'B';

        frame[GW_REQ_OFF_SOF] = GW_SOF_REQ;
        frame[GW_REQ_OFF_SEQ] = 0x43;
        frame[GW_REQ_OFF_OP] = GW_OP_ECHO;
        frame[GW_REQ_OFF_REG] = 0;
        gwWr16(&frame[GW_REQ_OFF_OFFSET], 0);
        gwWr16(&frame[GW_REQ_OFF_LEN], 1);
        frame[GW_REQ_HEADER_LEN] = a;
        size_t n = crc16ModbusAppend(frame, GW_REQ_HEADER_LEN + 1);
        st = stmLinkV2.sendRaw(frame, (uint16_t)n, 0x43, out, sizeof(out), got);
        bool first = (st == V2_OK && got == 1 && out[0] == 'A');

        frame[GW_REQ_HEADER_LEN] = b;
        n = crc16ModbusAppend(frame, GW_REQ_HEADER_LEN + 1);
        st = stmLinkV2.sendRaw(frame, (uint16_t)n, 0x43, out, sizeof(out), got);
        bool cached = (st == V2_OK && got == 1 && out[0] == 'A');

        check(first, "first request with SEQ 0x43 echoed 'A'");
        check(cached, "repeat of SEQ 0x43 returned the cached reply, not 'B'");

        // Same SEQ, different opcode. The cache must NOT answer this: reusing a
        // sequence number for a different command is a client bug, and serving
        // the old reply would hide it behind plausible-looking data.
        frame[GW_REQ_OFF_OP] = GW_OP_SYNC;
        gwWr16(&frame[GW_REQ_OFF_LEN], 0);
        n = crc16ModbusAppend(frame, GW_REQ_HEADER_LEN);
        st = stmLinkV2.sendRaw(frame, (uint16_t)n, 0x43, out, sizeof(out), got);
        check(st == V2_OK && got == sizeof(GwSyncRsp),
              "same SEQ with a different opcode is executed, not cached", resultText(st));
    }

    // --- and the link still works -----------------------------------------
    // The point of the whole section: none of the above wedged the parser. In
    // v1 a failed block read left a ghost frame in the buffer that was
    // re-executed on the next byte, and the client carried a resync() hack to
    // flush it.
    {
        GwSyncRsp s{};
        st = stmLinkV2.sync(s);
        check(st == V2_OK, "link still healthy after every error case", resultText(st));
    }
}

/* ==========================================================================
 * T8 - soak
 * ==========================================================================
 */
static void testSoak(uint32_t iterations) {
    banner("T8  Soak - latency and error rate");

    stmLinkV2.resetStats();

    uint8_t tx[64];
    uint8_t rx[64];
    for (uint16_t i = 0; i < sizeof(tx); i++) tx[i] = (uint8_t)i;

    uint32_t minUs = 0xFFFFFFFF, maxUs = 0, sumUs = 0, ok = 0;

    Serial.printf("       %lu iterations of a 64-byte echo...\r\n", (unsigned long)iterations);
    for (uint32_t i = 0; i < iterations; i++) {
        uint16_t got = 0;
        uint32_t t0 = micros();
        StmV2Status st = stmLinkV2.echo(tx, sizeof(tx), rx, sizeof(rx), got);
        uint32_t dt = micros() - t0;

        if (st == V2_OK && got == sizeof(tx) && memcmp(tx, rx, sizeof(tx)) == 0) {
            ok++;
            sumUs += dt;
            if (dt < minUs) minUs = dt;
            if (dt > maxUs) maxUs = dt;
        }
        if ((i % 50) == 49) Serial.print('.');
    }
    Serial.println();

    StmV2Stats s = stmLinkV2.stats();
    Serial.printf("       success      %lu / %lu\r\n", (unsigned long)ok,
                  (unsigned long)iterations);
    Serial.printf("       round trip   min %lu us, avg %lu us, max %lu us\r\n",
                  (unsigned long)(ok ? minUs : 0), (unsigned long)(ok ? sumUs / ok : 0),
                  (unsigned long)maxUs);
    Serial.printf("       errors       timeout %lu  crc %lu  frame %lu  seq %lu  device %lu\r\n",
                  (unsigned long)s.timeouts, (unsigned long)s.crcErrors,
                  (unsigned long)s.frameErrors, (unsigned long)s.seqErrors,
                  (unsigned long)s.deviceErrors);
    Serial.printf("       retries      %lu\r\n", (unsigned long)s.retries);

    check(ok == iterations, "no transaction lost");

    // 64 bytes each way at 115200 baud is about 12 ms of pure wire time, so
    // anything under 20 ms means the STM32 answered essentially immediately -
    // which is the claim the process image is making. v1 could take 1.6 s for
    // the same shape of request.
    if (ok > 0) {
        char d[48];
        snprintf(d, sizeof(d), "avg %lu us", (unsigned long)(sumUs / ok));
        check((sumUs / ok) < 20000, "average round trip under 20 ms", d);
    }
}

/* ==========================================================================
 * Live watch - the commissioning view
 * ==========================================================================
 */
static void liveWatch() {
    banner("Live watch - MB_RTU slots 0..7, any key to stop");

    if (!gMap.valid()) {
        StmV2Status st = stmLinkV2.readMeta(gMap);
        if (st != V2_OK) {
            Serial.printf("       need the map first: %s\r\n", resultText(st));
            return;
        }
    }

    while (!Serial.available()) {
        printSlots(GW_REGION_MB_RTU, 0, 8);
        Serial.println();
        delay(1000);
    }
    while (Serial.available()) (void)Serial.read();
}

/* ==========================================================================
 * Menu
 * ==========================================================================
 */
static void printMenu() {
    Serial.println();
    Serial.println("  ESP32 <-> STM32 link v2 test console");
    Serial.println("  ------------------------------------");
    Serial.println("   a  run every test and summarise      1  T1 echo");
    Serial.println("   l  live watch of MB_RTU slots        2  T2 sync");
    Serial.println("   k  soak, 200 transactions            3  T3 read meta");
    Serial.println("   s  show client statistics            4  T4 SYS region");
    Serial.println("   z  reset client statistics           5  T5 slots and quality");
    Serial.println("   ?  this menu                         6  T6 write region");
    Serial.println("                                        7  T7 error handling");
    Serial.println();
}

static void printStats() {
    StmV2Stats s = stmLinkV2.stats();
    banner("Client statistics");
    Serial.printf("       requests %lu  replies %lu  retries %lu\r\n", (unsigned long)s.requests,
                  (unsigned long)s.replies, (unsigned long)s.retries);
    Serial.printf("       timeout %lu  crc %lu  frame %lu  seq %lu  device %lu\r\n",
                  (unsigned long)s.timeouts, (unsigned long)s.crcErrors,
                  (unsigned long)s.frameErrors, (unsigned long)s.seqErrors,
                  (unsigned long)s.deviceErrors);
    Serial.printf("       last rtt %lu us  worst %lu us  last ok %lu ms ago\r\n",
                  (unsigned long)s.lastRttUs, (unsigned long)s.maxRttUs,
                  (unsigned long)(s.lastOkMs ? millis() - s.lastOkMs : 0));
}

static void runAll() {
    gPass = 0;
    gFail = 0;
    uint32_t t0 = millis();

    testEcho();
    testSync();
    testMeta();
    testSysRegion();
    testSlots();
    testWrite();
    testNegative();
    testSoak(200);

    banner("Summary");
    Serial.printf("       %lu passed, %lu failed, %lu ms\r\n", (unsigned long)gPass,
                  (unsigned long)gFail, (unsigned long)(millis() - t0));
    if (gFail == 0) {
        Serial.println("       Link v2 is good. Next step: the cloud back ends.");
    } else {
        Serial.println("       See docs/LINK_V2_TESTING.html for what each failure means.");
    }
}

/* ==========================================================================
 * Arduino entry points
 * ==========================================================================
 */
void setup() {
    Serial.begin(115200);
    delay(300);  // let the USB-serial console attach before the banner

    Serial.println();
    Serial.println("############################################################");
    Serial.println("#  IIoT gateway - inter-chip link v2 bring-up console       #");
    Serial.println("#  ESP32-S3 (network half)  <->  STM32F407 (industrial half)#");
    Serial.println("############################################################");
    Serial.printf("  protocol v%u.%u, payload cap %u bytes, %lu baud\r\n", GW_PROTO_VERSION_MAJOR,
                  GW_PROTO_VERSION_MINOR, GW_MAX_PAYLOAD, (unsigned long)STM_LINK_BAUD);
    Serial.printf("  UART1 rx=GPIO%d tx=GPIO%d   (console is UART0 on GPIO43/44)\r\n", PIN_STM_RX,
                  PIN_STM_TX);

    if (!stmLinkV2.begin()) {
        Serial.println("  link begin() FAILED - nothing below will work");
    }

    // Run the suite unattended once at boot, so plugging the board in is itself
    // a test. Press a key afterwards for anything else.
    runAll();
    printMenu();
}

void loop() {
    if (!Serial.available()) {
        delay(20);
        return;
    }

    int c = Serial.read();
    switch (c) {
        case 'a': runAll(); break;
        case '1': testEcho(); break;
        case '2': testSync(); break;
        case '3': testMeta(); break;
        case '4': testSysRegion(); break;
        case '5': testSlots(); break;
        case '6': testWrite(); break;
        case '7': testNegative(); break;
        case 'k': testSoak(200); break;
        case 'l': liveWatch(); break;
        case 's': printStats(); break;
        case 'z':
            stmLinkV2.resetStats();
            Serial.println("  statistics reset");
            break;
        case '?':
        case 'h': printMenu(); break;
        case '\r':
        case '\n': break;
        default: Serial.printf("  unknown key '%c' - press ? for the menu\r\n", (char)c); break;
    }
}
