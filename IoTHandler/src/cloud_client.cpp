#include "cloud_client.h"

#include <string.h>

#include "log.h"

CloudClient cloudClient;
CloudClient* CloudClient::_self = nullptr;

static const char* TAG = "cloud";

#if ENABLE_CLOUD

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>

#if !__has_include("secrets.h")
#error "include/secrets.h is missing. Copy include/secrets.h.example to include/secrets.h and fill it in, or build the 'linktest' environment which needs no credentials."
#endif

#include "process_image.h"
#include "secrets.h"
#include "stm_link.h"
#include "tag_map.h"

static WiFiClientSecure tlsClient;
static PubSubClient mqtt(tlsClient);

// Serialization scratch, sized against MQTT_BUFFER_BYTES. Static rather than on
// the task stack: a full snapshot of a large tag table would blow 8 KB.
static char jsonBuf[MQTT_BUFFER_BYTES];

CloudClient::CloudClient()
    : _cmdQueue(nullptr),
      _ackQueue(nullptr),
      _timeValid(false),
      _nextTelemetryMs(0),
      _nextStatusMs(0),
      _nextConnectMs(0),
      _backoffMs(RECONNECT_MIN_MS),
      _publishCount(0),
      _reconnectCount(0),
      _seq(0),
      _announced(false) {
    _topicTelemetry[0] = _topicStatus[0] = _topicCmd[0] = _topicAck[0] = '\0';
}

bool CloudClient::begin(QueueHandle_t cmdQueue, QueueHandle_t ackQueue) {
    _self = this;
    _cmdQueue = cmdQueue;
    _ackQueue = ackQueue;

    snprintf(_topicTelemetry, sizeof(_topicTelemetry), "%s/telemetry", DEVICE_ID);
    snprintf(_topicStatus, sizeof(_topicStatus), "%s/status", DEVICE_ID);
    snprintf(_topicCmd, sizeof(_topicCmd), "%s/cmd", DEVICE_ID);
    snprintf(_topicAck, sizeof(_topicAck), "%s/cmd/ack", DEVICE_ID);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // modem sleep adds hundreds of ms of publish latency
    WiFi.setAutoReconnect(true);

    tlsClient.setCACert(AWS_ROOT_CA);
    tlsClient.setCertificate(AWS_DEVICE_CERT);
    tlsClient.setPrivateKey(AWS_PRIVATE_KEY);

    mqtt.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
    mqtt.setCallback(trampoline);
    mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
    if (!mqtt.setBufferSize(MQTT_BUFFER_BYTES)) {
        LOGE(TAG, "could not allocate a %d byte MQTT buffer", MQTT_BUFFER_BYTES);
        return false;
    }

    LOGI(TAG, "endpoint %s:%d as %s", AWS_IOT_ENDPOINT, (int)AWS_IOT_PORT, DEVICE_ID);
    return true;
}

bool CloudClient::isConnected() const { return WiFi.isConnected() && mqtt.connected(); }

// ---------------------------------------------------------------------------
// Connection ladder: WiFi -> clock -> MQTT. Each rung is non-blocking and
// re-checked every loop, so losing any one of them recovers without a reboot.
// ---------------------------------------------------------------------------

bool CloudClient::ensureWifi() {
    if (WiFi.isConnected()) return true;

    // Re-issue begin() only after the association attempt has had its full
    // window; calling it repeatedly restarts the state machine and never settles.
    static uint32_t nextAttemptMs = 0;
    if (nextAttemptMs != 0 && (int32_t)(millis() - nextAttemptMs) < 0) return false;

    LOGI(TAG, "wifi connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    nextAttemptMs = millis() + WIFI_CONNECT_TIMEOUT_MS;
    return false;
}

bool CloudClient::ensureTime() {
    if (_timeValid) return true;

    static bool requested = false;
    if (!requested) {
        configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
        requested = true;
        LOGI(TAG, "waiting for SNTP - TLS rejects certificates without a real clock");
    }

    // Anything past 2023 means SNTP has landed.
    if (time(nullptr) > 1700000000L) {
        _timeValid = true;
        LOGI(TAG, "clock synced: %lu", (unsigned long)time(nullptr));
    }
    return _timeValid;
}

bool CloudClient::ensureMqtt() {
    if (mqtt.connected()) return true;
    if (millis() < _nextConnectMs) return false;

    JsonDocument will;
    will["dev"] = DEVICE_ID;
    will["online"] = false;
    will["reason"] = "lwt";
    serializeJson(will, jsonBuf, sizeof(jsonBuf));

    LOGI(TAG, "mqtt connecting (backoff %lu ms)", (unsigned long)_backoffMs);
    // AWS IoT Core does not accept retained messages, so willRetain is false.
    const bool ok = mqtt.connect(DEVICE_ID, _topicStatus, 0, false, jsonBuf);

    if (!ok) {
        // -2 (CONNECT_FAILED) here is almost always TLS: wrong endpoint, a
        // policy that denies iot:Connect for this client id, or certs that do
        // not match the Thing.
        LOGE(TAG, "mqtt connect failed, state=%d", mqtt.state());
        _nextConnectMs = millis() + _backoffMs;
        _backoffMs = _backoffMs * 2 > RECONNECT_MAX_MS ? RECONNECT_MAX_MS : _backoffMs * 2;
        return false;
    }

    _backoffMs = RECONNECT_MIN_MS;
    _reconnectCount++;
    _announced = false;
    mqtt.subscribe(_topicCmd, 1);
    LOGI(TAG, "mqtt up, subscribed %s", _topicCmd);
    return true;
}

void CloudClient::loop() {
    if (!ensureWifi()) return;
    if (!ensureTime()) return;
    if (!ensureMqtt()) return;

    mqtt.loop();

    if (!_announced) {
        publishStatus(true);
        _announced = true;
        _nextStatusMs = millis() + STATUS_INTERVAL_MS;
        _nextTelemetryMs = millis() + 1000;
    }

    drainAcks();

    const uint32_t now = millis();
    if ((int32_t)(now - _nextTelemetryMs) >= 0) {
        publishTelemetry();
        _nextTelemetryMs = now + TELEMETRY_INTERVAL_MS;
    }
    if ((int32_t)(now - _nextStatusMs) >= 0) {
        publishStatus(true);
        _nextStatusMs = now + STATUS_INTERVAL_MS;
    }
}

// ---------------------------------------------------------------------------
// Uplink
// ---------------------------------------------------------------------------

void CloudClient::publishTelemetry() {
    const uint16_t n = processImage.size();
    TagState* snap = (TagState*)malloc((size_t)n * sizeof(TagState));
    if (!snap) {
        LOGE(TAG, "no heap for a %u tag snapshot", (unsigned)n);
        return;
    }
    processImage.snapshot(snap);

    JsonDocument doc;
    doc["dev"] = DEVICE_ID;
    doc["ts"] = (uint32_t)time(nullptr);
    doc["seq"] = ++_seq;
    JsonArray arr = doc["tags"].to<JsonArray>();

    const uint32_t now = millis();
    for (uint16_t i = 0; i < n; ++i) {
        const TagDef& d = processImage.def(i);
        const TagState& s = snap[i];
        if (s.quality == Q_UNKNOWN) continue;  // never read - nothing to say yet

        JsonObject o = arr.add<JsonObject>();
        o["n"] = d.name;
        o["v"] = s.value;
        o["raw"] = s.raw;
        o["q"] = tagQualityName((TagQuality)s.quality);
        o["age"] = s.updatedMs ? (now - s.updatedMs) : 0;
        if (d.unit && d.unit[0]) o["u"] = d.unit;
    }
    free(snap);

    if (arr.size() == 0) return;

    const size_t len = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (len == 0 || len >= sizeof(jsonBuf) - 1) {
        LOGW(TAG, "telemetry too large for the %u byte buffer - trim the tag table "
                  "or raise MQTT_BUFFER_BYTES",
             (unsigned)sizeof(jsonBuf));
        return;
    }
    if (mqtt.publish(_topicTelemetry, (const uint8_t*)jsonBuf, len, false)) {
        _publishCount++;
        LOGD(TAG, "telemetry %u tags, %u bytes", (unsigned)arr.size(), (unsigned)len);
    } else {
        LOGW(TAG, "telemetry publish failed, state=%d", mqtt.state());
    }
}

void CloudClient::publishStatus(bool online) {
    const StmLinkStats st = stmLink.stats();

    JsonDocument doc;
    doc["dev"] = DEVICE_ID;
    doc["ts"] = (uint32_t)time(nullptr);
    doc["online"] = online;
    doc["upMs"] = millis();
    doc["heap"] = ESP.getFreeHeap();
    doc["rssi"] = WiFi.RSSI();
    doc["ip"] = WiFi.localIP().toString();
    doc["reconnects"] = _reconnectCount;
    doc["publishes"] = _publishCount;

    JsonObject link = doc["link"].to<JsonObject>();
    link["requests"] = st.requests;
    link["replies"] = st.replies;
    link["timeouts"] = st.timeouts;
    link["crcErrors"] = st.crcErrors;
    link["frameErrors"] = st.frameErrors;
    link["deviceErrors"] = st.deviceErrors;
    link["retries"] = st.retries;
    link["resyncs"] = st.resyncs;
    link["rttMs"] = st.lastRoundTripMs;
    link["sinceOkMs"] = st.lastOkMs ? (millis() - st.lastOkMs) : 0;
    link["tagsGood"] = processImage.goodCount();
    link["tagsTotal"] = processImage.size();

    const size_t len = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (len && !mqtt.publish(_topicStatus, (const uint8_t*)jsonBuf, len, false)) {
        LOGW(TAG, "status publish failed, state=%d", mqtt.state());
    }
}

void CloudClient::drainAcks() {
    LinkAck ack;
    while (_ackQueue && xQueueReceive(_ackQueue, &ack, 0) == pdTRUE) {
        JsonDocument doc;
        doc["dev"] = DEVICE_ID;
        doc["ts"] = (uint32_t)time(nullptr);
        doc["id"] = ack.id;
        doc["ok"] = (ack.accepted && ack.status == 0);
        doc["accepted"] = ack.accepted;
        doc["status"] = stmStatusName((StmStatus)ack.status);
        if (ack.tag[0]) doc["tag"] = ack.tag;
        if (ack.detail[0]) doc["detail"] = ack.detail;
        doc["raw"] = ack.written;

        const size_t len = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
        if (len) mqtt.publish(_topicAck, (const uint8_t*)jsonBuf, len, false);
        LOGI(TAG, "ack id=%s tag=%s status=%s", ack.id, ack.tag,
             stmStatusName((StmStatus)ack.status));
    }
}

// ---------------------------------------------------------------------------
// Downlink
// ---------------------------------------------------------------------------

void CloudClient::trampoline(char* topic, uint8_t* payload, unsigned int len) {
    if (_self) _self->onMessage(topic, payload, len);
}

void CloudClient::onMessage(const char* topic, const uint8_t* payload, unsigned int len) {
    LOGD(TAG, "rx %s (%u bytes)", topic, len);

    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, payload, len);
    if (err) {
        LOGW(TAG, "command is not valid JSON: %s", err.c_str());
        return;
    }

    LinkCommand cmd;
    memset(&cmd, 0, sizeof(cmd));

    const char* id = doc["id"] | "";
    strncpy(cmd.id, id, sizeof(cmd.id) - 1);

    const char* op = doc["op"] | "";
    if (strcmp(op, "ping") == 0) {
        cmd.kind = CMD_PING;
    } else if (strcmp(op, "write") == 0) {
        const char* tag = doc["tag"] | "";
        if (tagIndexOf(tag) < 0) {
            LOGW(TAG, "unknown tag '%s'", tag);
            return;
        }
        cmd.kind = CMD_WRITE_TAG;
        strncpy(cmd.tag, tag, sizeof(cmd.tag) - 1);
        cmd.value = doc["value"] | 0.0f;
        cmd.valueIsRaw = doc["raw"] | false;
    } else if (strcmp(op, "writeRaw") == 0) {
        cmd.kind = CMD_WRITE_RAW;
        cmd.regType = (uint8_t)(doc["type"] | 0);
        cmd.address = (uint16_t)(doc["addr"] | 0);
        cmd.value = doc["value"] | 0.0f;
        cmd.valueIsRaw = true;
        if (cmd.regType != STM_REG_HOLDING && cmd.regType != STM_REG_COIL) {
            LOGW(TAG, "writeRaw needs type 1 (holding) or 3 (coil), got %u",
                 (unsigned)cmd.regType);
            return;
        }
    } else {
        LOGW(TAG, "unknown op '%s'", op);
        return;
    }

    // Drop rather than block: this runs inside mqtt.loop() and stalling here
    // would stop the keepalive.
    if (!_cmdQueue || xQueueSend(_cmdQueue, &cmd, 0) != pdTRUE) {
        LOGW(TAG, "command queue full, dropped id=%s", cmd.id);
    }
}

#else  // ENABLE_CLOUD == 0

CloudClient::CloudClient()
    : _cmdQueue(nullptr),
      _ackQueue(nullptr),
      _timeValid(false),
      _nextTelemetryMs(0),
      _nextStatusMs(0),
      _nextConnectMs(0),
      _backoffMs(0),
      _publishCount(0),
      _reconnectCount(0),
      _seq(0),
      _announced(false) {
    _topicTelemetry[0] = _topicStatus[0] = _topicCmd[0] = _topicAck[0] = '\0';
}

bool CloudClient::begin(QueueHandle_t cmdQueue, QueueHandle_t ackQueue) {
    _self = this;
    _cmdQueue = cmdQueue;
    _ackQueue = ackQueue;
    LOGI(TAG, "disabled in this build (ENABLE_CLOUD=0) - link test only");
    return true;
}

void CloudClient::loop() {
    // Acks would otherwise pile up and block the link task's queue send.
    LinkAck ack;
    while (_ackQueue && xQueueReceive(_ackQueue, &ack, 0) == pdTRUE) {
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

bool CloudClient::isConnected() const { return false; }

void CloudClient::trampoline(char*, uint8_t*, unsigned int) {}
void CloudClient::onMessage(const char*, const uint8_t*, unsigned int) {}
bool CloudClient::ensureWifi() { return false; }
bool CloudClient::ensureTime() { return false; }
bool CloudClient::ensureMqtt() { return false; }
void CloudClient::publishTelemetry() {}
void CloudClient::publishStatus(bool) {}
void CloudClient::drainAcks() {}

#endif  // ENABLE_CLOUD
