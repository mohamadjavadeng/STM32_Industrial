/**
 * tb_main.cpp - the ThingsBoard gateway firmware.
 * ---------------------------------------------------------------------------
 *
 * One of three firmwares in src/, selected by build_src_filter in
 * platformio.ini. The others are main.cpp (AWS IoT, link v1, Modbus tags) and
 * linkv2_test_main.cpp (the bring-up console). Exactly one of them can be in a
 * build, because each defines setup() and loop().
 *
 * WHAT THIS ONE IS
 *   The smallest complete thing that is useful on a bench and honest in a
 *   panel: four relays and four inputs on the STM32, published to ThingsBoard
 *   and controllable from a dashboard, with every credential in NVS so that the
 *   binary is identical on every unit.
 *
 * WHY THERE IS NO RTOS TASK SPLIT HERE
 *   main.cpp runs the link on core 1 and the network on core 0, which is right
 *   when the link is doing a 1.6 second blocking Modbus proxy. Link v2 answers
 *   out of a process image in well under a millisecond, so the whole cycle -
 *   poll the STM32, service MQTT, service the console - fits comfortably in one
 *   loop and needs no queues, no mutexes and no core pinning.
 *
 *   The thing to watch, if this grows: stmLinkV2 transactions are synchronous
 *   and retry up to STMV2_MAX_ATTEMPTS, so a dead STM32 costs about 300 ms per
 *   poll. MQTT keepalive is 45 seconds, so that is survivable - but a second
 *   blocking consumer on this loop would not be, and that is the point at which
 *   the task split earns itself back.
 *
 * BOOT ORDER, AND WHY IT IS THIS WAY
 *   console -> config -> link -> local IO -> cloud
 *
 *   The console comes up first and unconditionally, so a unit that is
 *   misconfigured, or whose STM32 is not answering, can still be talked to and
 *   fixed. A commissioning tool that only works when everything else already
 *   works is not a commissioning tool.
 */
#include <Arduino.h>

#include "config.h"
#include "device_config.h"
#include "localio_client.h"
#include "log.h"
#include "provisioning.h"
#include "stm_link_v2.h"
#include "tb_client.h"

static const char* TAG = "main";

static bool gLinkOk;
static bool gIoOk;
static bool gCloudOk;
static uint32_t gNextHeartbeatMs;

/* One deadline per retry path. They used to share a single gNextRetryMs, so
 * whichever ran first pushed the other's deadline forward and the second could
 * be starved indefinitely - which, when the STM32 link is the thing that is
 * down, is exactly the retry you need to keep running. */
static uint32_t gNextIoRetryMs;
static uint32_t gNextCloudRetryMs;

/** Extra lines for the console's STATUS command. */
static void statusHook() {
    char buf[48];
    StmV2Stats s = stmLinkV2.stats();

    provEmit("link.ready", stmLinkV2.isReady() ? "1" : "0");
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s.requests);
    provEmit("link.requests", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s.timeouts);
    provEmit("link.timeouts", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s.crcErrors);
    provEmit("link.crcErrors", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s.lastRttUs);
    provEmit("link.lastRttUs", buf);

    provEmit("io.ready", localIo.isReady() ? "1" : "0");
    if (localIo.isReady()) {
        LocalIoState st = localIo.state();
        snprintf(buf, sizeof(buf), "%u", (unsigned)st.relayMask);
        provEmit("io.relayMask", buf);
        snprintf(buf, sizeof(buf), "%u", (unsigned)st.inputMask);
        provEmit("io.inputMask", buf);
        provEmit("io.quality", gwQualityName(st.quality));
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)localIo.errorCount());
        provEmit("io.pollErrors", buf);
    }

    tbClient.reportStatus();
}

/**
 * Brings up the STM32 link and the local IO cache.
 *
 * Separate from setup() because it is also the retry path: if the STM32 is
 * still booting, or is flashed with GW_LINK_ENABLE 0, or is simply not wired
 * yet, the gateway keeps running and tries again rather than sitting there
 * needing a power cycle to notice the cable was plugged in.
 */
static bool bringUpLink() {
    GwSyncRsp id;

    StmV2Status st = stmLinkV2.sync(id);
    if (st != V2_OK) {
        LOGW(TAG, "STM32 SYNC failed: %s", stmV2StatusName(st));
        return false;
    }

    LOGI(TAG, "STM32 fw %u.%u.%u proto 0x%02X uptime %lu ms caps 0x%04X", id.fwMajor, id.fwMinor,
         id.fwPatch, id.protoVersion, (unsigned long)id.stmUptimeMs, id.capabilities);

    /* A major-version mismatch is refused rather than worked around. Two sides
     * of a binary protocol that disagree still parse each other's frames - they
     * just put the values in the wrong places, and nothing downstream can tell.
     * See the versioning note at the top of shared/gw_model.h. */
    if ((id.protoVersion >> 4) != GW_PROTO_VERSION_MAJOR) {
        LOGE(TAG, "protocol major mismatch: STM32 0x%02X, we expect %u.x - reflash both chips",
             id.protoVersion, GW_PROTO_VERSION_MAJOR);
        return false;
    }

    if ((id.capabilities & GW_CAP_LOCAL_IO) == 0) {
        /* Reads will still work. Relay commands will come back NOT_IMPL, so say
         * so now instead of letting every dashboard click fail silently. */
        LOGW(TAG, "STM32 does not advertise LOCAL_IO - relays will not switch. "
                  "Build it with GW_LOCALIO_ENABLE 1.");
    }

    if (!localIo.begin(stmLinkV2)) {
        LOGE(TAG, "local IO unavailable");
        return false;
    }

    LOGI(TAG, "local IO ready: relays=0x%X inputs=0x%X", localIo.state().relayMask,
         localIo.state().inputMask);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(200); /* let the USB-serial bridge enumerate before the banner */

    Serial.print("\r\n\r\n");
    LOGI(TAG, "=== IoTPLC gateway - ThingsBoard / local IO - %s ===", GW_FW_STRING);
    LOGI(TAG, "relays PD0..PD3, inputs PD4..PD7 (pull-up) on the STM32");

    deviceConfig.load();
    provisioningBegin(statusHook);

    if (!deviceConfig.isProvisioned()) {
        LOGW(TAG, "UNPROVISIONED - run: python tools/gw_config_tool.py");
    }

    gLinkOk = stmLinkV2.begin();
    if (!gLinkOk) {
        LOGE(TAG, "UART to the STM32 would not open");
    } else {
        gIoOk = bringUpLink();
    }

    gCloudOk = tbClient.begin(&localIo);

#if STATUS_LED_ENABLE
    pinMode(RGB_BUILTIN, OUTPUT);
#endif

    LOGI(TAG, "boot complete: link=%d io=%d cloud=%d", (int)gLinkOk, (int)gIoOk, (int)gCloudOk);
}

void loop() {
    uint32_t now = millis();

    /* First and unconditional. Whatever else is broken, the console answers. */
    provisioningPoll();

    if (gIoOk) {
        localIo.poll(LOCALIO_POLL_MS);
    } else if (gLinkOk && (int32_t)(now - gNextIoRetryMs) >= 0) {
        /* Retry slowly. The usual cause is an STM32 that is still booting or
         * one that has just been reflashed, and both resolve themselves within
         * a few seconds; hammering SYNC at full speed in the meantime would
         * just fill the log. */
        gNextIoRetryMs = now + 5000;
        gIoOk = bringUpLink();
    }

    if (gCloudOk) {
        tbClient.loop();
    } else if (deviceConfig.isProvisioned() && (int32_t)(now - gNextCloudRetryMs) >= 0) {
        /* Credentials can arrive over the console while the gateway is running,
         * so a unit that booted unprovisioned picks them up on the next SAVE
         * without needing the REBOOT the tool offers. */
        gNextCloudRetryMs = now + 5000;
        gCloudOk = tbClient.begin(&localIo);
    }

#if STATUS_LED_ENABLE
    if ((int32_t)(now - gNextHeartbeatMs) >= 0) {
        gNextHeartbeatMs = now + HEARTBEAT_MS;

        /* Colour is the fastest diagnosis available with the lid off:
         *   green  - publishing to ThingsBoard
         *   blue   - STM32 IO is alive, cloud is not
         *   red    - the STM32 link is down
         *   yellow - unprovisioned, waiting for the config tool */
        uint8_t r = 0, g = 0, b = 0;
        if (!deviceConfig.isProvisioned()) {
            r = 8;
            g = 8;
        } else if (!gIoOk) {
            r = 12;
        } else if (tbClient.isConnected()) {
            g = 12;
        } else {
            b = 12;
        }
        neopixelWrite(RGB_BUILTIN, r, g, b);
    }
#endif
}
