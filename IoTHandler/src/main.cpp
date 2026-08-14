/**
 * IIoT gateway - ESP32-S3 network half.
 *
 * Pairs with the STM32F407 industrial half in ../ModbusRTU_AWS. The STM32 owns
 * the process: it is the Modbus RTU master on RS-485 and runs the IO. This chip
 * owns connectivity only.
 *
 * Split of duties
 *   STM32   authoritative process data, control loop, field bus timing
 *   ESP32   WiFi, TLS, MQTT, buffering, cloud cadence, downlink commands
 *
 * Direction of control on the inter-chip link: the ESP32 is the initiator and
 * the STM32 is a pure responder. The data owner answers, it does not push. That
 * way a rebooting or wedged radio can never stall the control loop, and the side
 * that knows whether it is connected is the side that decides the poll rate.
 *
 * Task layout
 *   core 1  linkTask  owns Serial1, polls tags into the process image, executes
 *                     queued write commands. Deterministic, never touches WiFi.
 *   core 0  netTask   WiFi + TLS + MQTT. Publishes from process image snapshots.
 *   core 1  loopTask  Arduino loop(): heartbeat LED, supervisor, console stats.
 *
 * The two tasks never share the serial port or the MQTT client; they exchange
 * fixed-size structs through two FreeRTOS queues.
 */
#include <Arduino.h>
#include <freertos/queue.h>
#include <stdio.h>
#include <string.h>

#include "cloud_client.h"
#include "commands.h"
#include "config.h"
#include "log.h"
#include "process_image.h"
#include "stm_link.h"
#include "stm_protocol.h"
#include "tag_map.h"

static const char* TAG = "main";

static QueueHandle_t cmdQueue = nullptr;
static QueueHandle_t ackQueue = nullptr;
static TaskHandle_t linkTaskHandle = nullptr;
static TaskHandle_t netTaskHandle = nullptr;

// Supervisor check-ins. Written by the workers, read by loop().
static volatile uint32_t linkAliveMs = 0;
static volatile uint32_t netAliveMs = 0;

// Per-tag next-due schedule, parallel to the tag table.
static uint32_t* tagNextPollMs = nullptr;

// ---------------------------------------------------------------------------
// Optional STM32 -> ESP32 attention line
// ---------------------------------------------------------------------------
#if PIN_STM_EVENT >= 0
static void IRAM_ATTR onStmEvent() {
    if (linkTaskHandle) vTaskNotifyGiveFromISR(linkTaskHandle, nullptr);
}
#endif

// ---------------------------------------------------------------------------
// Status LED (Freenove board: WS2812 on GPIO48)
// ---------------------------------------------------------------------------
static void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
#if STATUS_LED_ENABLE && defined(RGB_BUILTIN)
    neopixelWrite(RGB_BUILTIN, r, g, b);
#else
    (void)r;
    (void)g;
    (void)b;
#endif
}

// ---------------------------------------------------------------------------
// Poll engine
// ---------------------------------------------------------------------------

/** Reads one tag and files the result in the process image. */
static void pollTag(uint16_t i) {
    const TagDef& d = processImage.def(i);
    uint16_t raw = 0;
    const StmStatus st = stmLink.readRegister(d.regType, d.address, raw);

    if (st == STM_OK) {
        processImage.update(i, raw);
        LOGD(TAG, "%s = %u", d.name, (unsigned)raw);
    } else {
        processImage.fail(i, (uint8_t)st);
        LOGW(TAG, "read %s (%s %u) failed: %s", d.name, stmRegTypeName(d.regType),
             (unsigned)d.address, stmStatusName(st));
    }
}

/** Converts an engineering value from a command into a raw register value. */
static bool engineeringToRaw(const TagDef& d, float value, bool valueIsRaw, uint16_t& raw,
                            char* detail, size_t detailLen) {
    if (valueIsRaw) {
        if (value < 0.0f || value > 65535.0f) {
            snprintf(detail, detailLen, "raw %.0f out of range", value);
            return false;
        }
        raw = (uint16_t)(value + 0.5f);
        return true;
    }

    if (d.regType == STM_REG_COIL) {
        raw = (value != 0.0f) ? 1 : 0;
        return true;
    }

    if (d.scale == 0.0f) {
        snprintf(detail, detailLen, "tag scale is zero");
        return false;
    }

    const float scaled = (value - d.offset) / d.scale;
    const float rounded = scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f;

    if (d.isSigned) {
        if (rounded < -32768.0f || rounded > 32767.0f) {
            snprintf(detail, detailLen, "%.3f -> %.0f out of int16", value, scaled);
            return false;
        }
        raw = (uint16_t)(int16_t)rounded;
    } else {
        if (rounded < 0.0f || rounded > 65535.0f) {
            snprintf(detail, detailLen, "%.3f -> %.0f out of uint16", value, scaled);
            return false;
        }
        raw = (uint16_t)rounded;
    }
    return true;
}

static void executeCommand(const LinkCommand& cmd) {
    LinkAck ack;
    memset(&ack, 0, sizeof(ack));
    memcpy(ack.id, cmd.id, sizeof(ack.id));
    memcpy(ack.tag, cmd.tag, sizeof(ack.tag));
    ack.kind = cmd.kind;
    ack.accepted = true;

    switch (cmd.kind) {
        case CMD_PING: {
            // Probe the first tag in the table; its address is known to respond.
            const TagDef& d = processImage.def(0);
            uint16_t raw = 0;
            ack.status = (uint8_t)stmLink.readRegister(d.regType, d.address, raw);
            ack.written = raw;
            snprintf(ack.detail, sizeof(ack.detail), "%s@%u", stmRegTypeName(d.regType),
                     (unsigned)d.address);
            break;
        }

        case CMD_WRITE_TAG: {
            const int16_t idx = tagIndexOf(cmd.tag);
            if (idx < 0) {
                ack.accepted = false;
                ack.status = (uint8_t)STM_ERR_BAD_ARG;
                snprintf(ack.detail, sizeof(ack.detail), "unknown tag");
                break;
            }
            const TagDef& d = processImage.def((uint16_t)idx);
            if (!d.writable) {
                ack.accepted = false;
                ack.status = (uint8_t)STM_ERR_BAD_ARG;
                snprintf(ack.detail, sizeof(ack.detail), "tag is read-only");
                break;
            }
            uint16_t raw = 0;
            if (!engineeringToRaw(d, cmd.value, cmd.valueIsRaw, raw, ack.detail,
                                  sizeof(ack.detail))) {
                ack.accepted = false;
                ack.status = (uint8_t)STM_ERR_BAD_ARG;
                break;
            }
            ack.written = raw;
#if WRITE_VERIFY
            ack.status = (uint8_t)stmLink.writeRegisterVerified(d.regType, d.address, raw);
            if (ack.status == STM_ERR_DEVICE) {
                snprintf(ack.detail, sizeof(ack.detail), "read-back mismatch");
            }
#else
            ack.status = (uint8_t)stmLink.writeRegister(d.regType, d.address, raw);
#endif
            // Refresh the cache so the next publish reflects the new value.
            if (ack.status == STM_OK) pollTag((uint16_t)idx);
            break;
        }

        case CMD_WRITE_RAW: {
            if (cmd.value < 0.0f || cmd.value > 65535.0f) {
                ack.accepted = false;
                ack.status = (uint8_t)STM_ERR_BAD_ARG;
                snprintf(ack.detail, sizeof(ack.detail), "value out of range");
                break;
            }
            const uint16_t raw = (uint16_t)(cmd.value + 0.5f);
            ack.written = raw;
            snprintf(ack.detail, sizeof(ack.detail), "%s@%u", stmRegTypeName(cmd.regType),
                     (unsigned)cmd.address);
#if WRITE_VERIFY
            ack.status = (uint8_t)stmLink.writeRegisterVerified(cmd.regType, cmd.address, raw);
#else
            ack.status = (uint8_t)stmLink.writeRegister(cmd.regType, cmd.address, raw);
#endif
            break;
        }

        default:
            ack.accepted = false;
            ack.status = (uint8_t)STM_ERR_BAD_ARG;
            snprintf(ack.detail, sizeof(ack.detail), "unknown kind");
            break;
    }

    if (ackQueue && xQueueSend(ackQueue, &ack, 0) != pdTRUE) {
        LOGW(TAG, "ack queue full, dropped id=%s", ack.id);
    }
}

static void linkTask(void*) {
    // Commands take priority over polling: an operator waiting on a setpoint
    // should not queue behind a full scan.
    for (;;) {
        linkAliveMs = millis();

        LinkCommand cmd;
        while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) {
            executeCommand(cmd);
            linkAliveMs = millis();
        }

        const uint32_t now = millis();
        for (uint16_t i = 0; i < processImage.size(); ++i) {
            if ((int32_t)(now - tagNextPollMs[i]) < 0) continue;
            pollTag(i);
            tagNextPollMs[i] = millis() + processImage.def(i).pollMs;
            linkAliveMs = millis();

            // Re-check the command queue between tags so a write never waits for
            // the rest of the scan.
            if (uxQueueMessagesWaiting(cmdQueue) > 0) break;
        }

        processImage.ageTags();

        // A command arrived mid-scan: go straight back round, do not sleep.
        if (uxQueueMessagesWaiting(cmdQueue) > 0) continue;

        // Sleep until the next poll is due or the attention line fires.
        // ulTaskNotifyTake is the sleep, so an ISR notify wakes us immediately.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25));
    }
}

static void netTask(void*) {
    cloudClient.begin(cmdQueue, ackQueue);
    for (;;) {
        netAliveMs = millis();
        cloudClient.loop();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(200);  // let the USB-serial adapter attach before the banner

    Serial.println();
    LOGI(TAG, "=== IIoT gateway, ESP32-S3 network half ===");
    LOGI(TAG, "reset reason %d, heap %lu, cloud %s", (int)esp_reset_reason(),
         (unsigned long)ESP.getFreeHeap(), ENABLE_CLOUD ? "enabled" : "DISABLED (link test)");

    setStatusLed(16, 0, 0);  // red until the link answers

    if (!processImage.begin()) {
        LOGE(TAG, "process image init failed");
        delay(2000);
        ESP.restart();
    }

    tagNextPollMs = (uint32_t*)calloc(processImage.size(), sizeof(uint32_t));
    if (!tagNextPollMs) {
        LOGE(TAG, "no heap for the poll schedule");
        delay(2000);
        ESP.restart();
    }
    // Stagger the first reads so the whole table does not hit the link at once.
    for (uint16_t i = 0; i < processImage.size(); ++i) {
        tagNextPollMs[i] = millis() + (uint32_t)i * 25;
    }

    if (!stmLink.begin(Serial1)) {
        LOGE(TAG, "STM32 link init failed");
        delay(2000);
        ESP.restart();
    }

#if PIN_STM_EVENT >= 0
    attachInterrupt(digitalPinToInterrupt(PIN_STM_EVENT), onStmEvent, FALLING);
    LOGI(TAG, "attention line on GPIO%d", PIN_STM_EVENT);
#endif

    cmdQueue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(LinkCommand));
    ackQueue = xQueueCreate(ACK_QUEUE_DEPTH, sizeof(LinkAck));
    if (!cmdQueue || !ackQueue) {
        LOGE(TAG, "queue allocation failed");
        delay(2000);
        ESP.restart();
    }

    // First contact. Not fatal - the STM32 may still be booting, or its RS-485
    // slave may be absent, and the poll loop retries forever anyway.
    const StmStatus probe = stmLink.ping(processImage.def(0).address);
    LOGI(TAG, "probe %s@%u -> %s", stmRegTypeName(processImage.def(0).regType),
         (unsigned)processImage.def(0).address, stmStatusName(probe));

    xTaskCreatePinnedToCore(linkTask, "link", POLL_TASK_STACK, nullptr, POLL_TASK_PRIO,
                            &linkTaskHandle, POLL_TASK_CORE);
    xTaskCreatePinnedToCore(netTask, "net", NET_TASK_STACK, nullptr, NET_TASK_PRIO,
                            &netTaskHandle, NET_TASK_CORE);

    LOGI(TAG, "running");
}

void loop() {
    static uint32_t nextBeatMs = 0;
    static uint32_t nextStatsMs = 0;
    static bool beatOn = false;

    const uint32_t now = millis();

    if ((int32_t)(now - nextBeatMs) >= 0) {
        nextBeatMs = now + HEARTBEAT_MS;
        beatOn = !beatOn;

        // Green: link and cloud both healthy. Blue: link fine, cloud not
        // connected (or disabled). Red: the STM32 has not answered recently.
        const StmLinkStats st = stmLink.stats();
        const bool linkOk = st.lastOkMs != 0 && (now - st.lastOkMs) < 3 * TAG_STALE_MS;
        const uint8_t level = beatOn ? 24 : 2;
        if (!linkOk) {
            setStatusLed(level, 0, 0);
        } else if (cloudClient.isConnected()) {
            setStatusLed(0, level, 0);
        } else {
            setStatusLed(0, 0, level);
        }
    }

    if ((int32_t)(now - nextStatsMs) >= 0) {
        nextStatsMs = now + 15000;
        const StmLinkStats st = stmLink.stats();
        LOGI(TAG,
             "link req=%lu rsp=%lu to=%lu crc=%lu frame=%lu dev=%lu retry=%lu resync=%lu "
             "rtt=%lums | tags %u/%u good | heap %lu",
             (unsigned long)st.requests, (unsigned long)st.replies, (unsigned long)st.timeouts,
             (unsigned long)st.crcErrors, (unsigned long)st.frameErrors,
             (unsigned long)st.deviceErrors, (unsigned long)st.retries,
             (unsigned long)st.resyncs, (unsigned long)st.lastRoundTripMs,
             (unsigned)processImage.goodCount(), (unsigned)processImage.size(),
             (unsigned long)ESP.getFreeHeap());
    }

#if SUPERVISOR_ENABLE
    // A gateway that has silently stopped reporting is worse than one that
    // reboots. Both workers stamp a timestamp every iteration.
    if (linkAliveMs != 0 && (now - linkAliveMs) > SUPERVISOR_TIMEOUT_MS) {
        LOGE(TAG, "link task stalled for %lu ms - restarting",
             (unsigned long)(now - linkAliveMs));
        delay(100);
        ESP.restart();
    }
    if (netAliveMs != 0 && (now - netAliveMs) > SUPERVISOR_TIMEOUT_MS) {
        LOGE(TAG, "net task stalled for %lu ms - restarting", (unsigned long)(now - netAliveMs));
        delay(100);
        ESP.restart();
    }
#endif

    delay(50);
}
