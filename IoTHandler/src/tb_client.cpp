/**
 * tb_client.cpp - ThingsBoard MQTT. See tb_client.h.
 */
#include "tb_client.h"

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "device_config.h"
#include "log.h"
#include "provisioning.h"

static const char* TAG = "tb";

/**
 * PubSubClient's state() as a word.
 *
 * The raw code is a small negative or positive integer and the two that matter
 * most look like nothing: 4 is BAD_CREDENTIALS and 5 is UNAUTHORIZED, which on
 * ThingsBoard both mean the access token is wrong. Printing "state=5" made the
 * single most common commissioning mistake unreadable.
 */
static const char* mqttStateName(int s) {
    switch (s) {
        case -4: return "CONNECTION_TIMEOUT";
        case -3: return "CONNECTION_LOST";
        case -2: return "CONNECT_FAILED";
        case -1: return "DISCONNECTED";
        case 0: return "CONNECTED";
        case 1: return "BAD_PROTOCOL";
        case 2: return "BAD_CLIENT_ID";
        case 3: return "UNAVAILABLE";
        case 4: return "BAD_CREDENTIALS";
        case 5: return "UNAUTHORIZED";
        default: return "?";
    }
}

static const char* TOPIC_TELEMETRY = "v1/devices/me/telemetry";
static const char* TOPIC_ATTRIBUTES = "v1/devices/me/attributes";
static const char* TOPIC_RPC_SUB = "v1/devices/me/rpc/request/+";
static const char* TOPIC_RPC_PREFIX = "v1/devices/me/rpc/request/";

/* Both transports are constructed; only one is handed to PubSubClient, chosen
 * at begin() from the stored config. A WiFiClientSecure costs a few hundred
 * bytes until it is used, which is cheaper than the alternative of allocating
 * one on the heap and dealing with the lifetime. */
static WiFiClient gPlainClient;
static WiFiClientSecure gTlsClient;
static PubSubClient gMqtt;

TbClient tbClient;
TbClient* TbClient::_self = nullptr;

TbClient::TbClient()
    : _io(nullptr),
      _nextTelemetryMs(0),
      _nextConnectMs(0),
      _nextAttributesMs(0),
      _nextTraceMs(0),
      _nextSkipLogMs(0),
      _backoffMs(RECONNECT_MIN_MS),
      _wifiStartedMs(0),
      _publishCount(0),
      _reconnectCount(0),
      _rpcCount(0),
      _lastRelayMask(0),
      _lastInputMask(0),
      _everPublished(false),
      _attributesSent(false) {}

void TbClient::trampoline(char* topic, uint8_t* payload, unsigned int len) {
    if (_self) _self->onMessage(topic, payload, len);
}

bool TbClient::begin(LocalIoClient* io) {
    const DeviceConfig& c = deviceConfig.config();

    _self = this;
    _io = io;

    if (!deviceConfig.isProvisioned()) {
        LOGW(TAG, "not provisioned - run tools/gw_config_tool.py and set wifi.ssid, "
                  "tb.host and tb.token");
        return false;
    }

    if (c.tbUseTls) {
        /* Certificate validation is OFF until a CA is provisioned.
         *
         * Said loudly rather than quietly, because unvalidated TLS encrypts the
         * link but proves nothing about who is on the other end of it: anything
         * that can intercept the connection can present its own certificate and
         * collect the access token, which is the whole of this device's
         * identity. It is better than plaintext on an untrusted network and
         * materially weaker than real TLS, and an operator who was not told
         * would reasonably assume the latter.
         *
         * To fix: call gTlsClient.setCACert() with your server's issuer before
         * connecting. ThingsBoard Cloud and demo.thingsboard.io use ISRG Root
         * X1 (Let's Encrypt); a self-hosted server uses whatever you issued it.
         */
        gTlsClient.setInsecure();
        gMqtt.setClient(gTlsClient);
        LOGW(TAG, "TLS enabled WITHOUT certificate validation - see tb_client.cpp");
    } else {
        gMqtt.setClient(gPlainClient);
    }

    gMqtt.setServer(c.tbHost, c.tbPort);
    gMqtt.setCallback(trampoline);
    gMqtt.setKeepAlive(MQTT_KEEPALIVE_S);

    /* PubSubClient's stock 256-byte buffer silently drops any publish larger
     * than itself and returns false, which looks exactly like a network fault.
     * A telemetry snapshot with all eight channels and diagnostics does not
     * fit in 256. */
    if (!gMqtt.setBufferSize(MQTT_BUFFER_BYTES)) {
        LOGE(TAG, "could not allocate a %u byte MQTT buffer", (unsigned)MQTT_BUFFER_BYTES);
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false); /* modem sleep adds seconds of latency to an RPC */

    LOGI(TAG, "target %s:%u tls=%d device='%s'", c.tbHost, (unsigned)c.tbPort, (int)c.tbUseTls,
         c.deviceName);
    return true;
}

bool TbClient::ensureWifi() {
    const DeviceConfig& c = deviceConfig.config();

    if (WiFi.status() == WL_CONNECTED) return true;

    uint32_t now = millis();

    if (_wifiStartedMs == 0 || (uint32_t)(now - _wifiStartedMs) > WIFI_CONNECT_TIMEOUT_MS) {
        LOGI(TAG, "connecting to WiFi '%s'", c.wifiSsid);
        WiFi.disconnect();
        WiFi.begin(c.wifiSsid, c.wifiPass[0] ? c.wifiPass : nullptr);
        _wifiStartedMs = now == 0 ? 1 : now;
    }
    return false;
}

bool TbClient::ensureMqtt() {
    const DeviceConfig& c = deviceConfig.config();

    if (gMqtt.connected()) return true;

    uint32_t now = millis();
    if ((int32_t)(now - _nextConnectMs) < 0) return false;

    LOGI(TAG, "MQTT connect as '%s'", c.deviceName);

    /* ThingsBoard identifies the device by the MQTT username, which is the
     * access token. The client id only has to be unique on the broker, and the
     * password is unused with token auth. */
    bool ok = gMqtt.connect(c.deviceName, c.tbToken, nullptr);

    if (!ok) {
        /* Exponential backoff, capped. A device that retries every two seconds
         * against a broker that is rejecting its token is just noise in someone
         * else's log, and ThingsBoard rate-limits it anyway. */
        _backoffMs = _backoffMs * 2;
        if (_backoffMs > RECONNECT_MAX_MS) _backoffMs = RECONNECT_MAX_MS;
        _nextConnectMs = now + _backoffMs;
        int state = gMqtt.state();
        LOGW(TAG, "MQTT connect failed: %s (%d), retry in %lu ms", mqttStateName(state), state,
             (unsigned long)_backoffMs);
        if (state == 4 || state == 5) {
            LOGE(TAG, "the broker rejected our credentials - tb.token is wrong, or the "
                      "device was deleted in ThingsBoard");
        }
        return false;
    }

    _backoffMs = RECONNECT_MIN_MS;
    ++_reconnectCount;
    _attributesSent = false;
    LOGI(TAG, "MQTT connected");

    if (!gMqtt.subscribe(TOPIC_RPC_SUB, 1)) {
        /* Without this subscription the dashboard's switches do nothing and
         * nothing says why, so treat it as a failed connection and retry
         * rather than sitting there publishing happily. */
        LOGE(TAG, "RPC subscribe failed - disconnecting to retry");
        gMqtt.disconnect();
        return false;
    }

    publishAttributes();
    return true;
}

void TbClient::publishAttributes() {
    JsonDocument doc;
    const DeviceConfig& c = deviceConfig.config();

    doc["fwVersion"] = GW_FW_STRING;
    doc["deviceName"] = c.deviceName;
    doc["mac"] = WiFi.macAddress();
    doc["ip"] = WiFi.localIP().toString();
    doc["protoVersion"] = GW_PROTO_VERSION;
    doc["relayCount"] = GW_LIO_DO_COUNT;
    doc["inputCount"] = GW_LIO_DI_COUNT;

    if (_io && _io->isReady()) {
        doc["stmConfigCrc"] = _io->map().configCrc32;
        doc["stmProtoVersion"] = _io->map().protoVersion;
        doc["localIoReady"] = true;
    } else {
        /* Published as an attribute rather than only logged, so that a unit
         * whose STM32 link is dead is visible in the device list instead of
         * looking like a device that simply has not reported yet. */
        doc["localIoReady"] = false;
    }

    char buf[512];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n == 0 || n >= sizeof(buf)) {
        LOGE(TAG, "attribute payload did not fit");
        return;
    }

    if (gMqtt.publish(TOPIC_ATTRIBUTES, buf, false)) {
        _attributesSent = true;
#if CLOUD_TRACE
        LOGI(TAG, "PUB %s  %u bytes  ok", TOPIC_ATTRIBUTES, (unsigned)n);
#endif
    } else {
        /* Back off before trying again.
         *
         * This used to be retried from loop() with no delay whenever the flag
         * was still clear, which on a single failure became a publish attempt
         * every loop iteration - thousands per second. ThingsBoard rate-limits
         * a device that does that and then drops the connection, so one failed
         * attribute publish could take the whole uplink down and look exactly
         * like "the gateway stopped reporting". */
        _nextAttributesMs = millis() + 5000;
        LOGW(TAG, "attribute publish failed (%s) - retry in 5 s",
             mqttStateName(gMqtt.state()));
    }
}

void TbClient::publishTelemetry() {
    if (!gMqtt.connected()) {
        /* Previously a silent return. A gateway that has quietly stopped
         * publishing because the broker went away looks identical, from the
         * console, to one that is publishing fine - which is precisely the
         * confusion this whole class of bug lives in. Throttled so a long
         * outage does not bury the rest of the log. */
        uint32_t now = millis();
        if ((int32_t)(now - _nextSkipLogMs) >= 0) {
            _nextSkipLogMs = now + 5000;
            LOGW(TAG, "telemetry skipped - MQTT %s", mqttStateName(gMqtt.state()));
        }
        return;
    }

    JsonDocument doc;

    /* ioReady is published either way.
     *
     * The previous version simply left relay1..4 and input1..4 out of the
     * payload when the local IO was not available, which is why a dashboard
     * could sit there with no input keys at all and nothing to explain it -
     * an absent key looks identical to a key that was never configured. The
     * values are still withheld when they are not known, because publishing
     * a guessed false is the one thing this design refuses to do; what gets
     * published instead is the reason. */
    doc["ioReady"] = (_io != nullptr && _io->isReady());

    if (_io && _io->isReady()) {
        LocalIoState s = _io->state();

        for (uint8_t i = 0; i < GW_LIO_DO_COUNT; ++i) {
            char key[12];
            snprintf(key, sizeof(key), "relay%u", (unsigned)(i + 1));
            doc[key] = (bool)((s.relayMask >> i) & 1u);
        }
        for (uint8_t i = 0; i < GW_LIO_DI_COUNT; ++i) {
            char key[12];
            snprintf(key, sizeof(key), "input%u", (unsigned)(i + 1));
            doc[key] = (bool)((s.inputMask >> i) & 1u);
        }

        doc["relayMask"] = s.relayMask;
        doc["inputMask"] = s.inputMask;

        /* Quality travels with the values, always. A stale mask published as a
         * plain number is indistinguishable from a fresh one, and a dashboard
         * has no way to know it is looking at history. */
        doc["ioQuality"] = gwQualityName(s.quality);
        doc["ioQualityCode"] = s.quality;
        doc["ioValid"] = s.valid;
        doc["ioAgeMs"] = (uint32_t)(millis() - s.sampledMs);
        doc["stmStampMs"] = s.stmStampMs;

        _lastRelayMask = s.relayMask;
        _lastInputMask = s.inputMask;
    } else {
        doc["ioValid"] = false;
        doc["ioQuality"] = gwQualityName(GW_Q_COMM_FAIL);
        doc["ioError"] = stmLinkV2.isReady() ? "STM32 link up, LOCAL_IO region unavailable"
                                             : "no UART to the STM32";
    }

    StmV2Stats st = stmLinkV2.stats();
    doc["linkOk"] = (st.lastOkMs != 0) && ((millis() - st.lastOkMs) < 5000);
    doc["linkRttUs"] = st.lastRttUs;
    doc["linkTimeouts"] = st.timeouts;
    doc["linkCrcErrors"] = st.crcErrors;
    doc["linkRetries"] = st.retries;
    doc["rssi"] = WiFi.RSSI();
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["uptimeSec"] = millis() / 1000UL;

    char buf[1024];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n == 0 || n >= sizeof(buf)) {
        LOGE(TAG, "telemetry payload did not fit");
        return;
    }

    if (gMqtt.publish(TOPIC_TELEMETRY, buf, false)) {
        ++_publishCount;
        _everPublished = true;
#if CLOUD_TRACE
        /* The payload verbatim. This is the line that settles "is the gateway
         * sending it" versus "is the cloud showing it" without guesswork: if
         * input1 is in here and the dashboard is empty, the firmware is done
         * and the problem is the device binding or the widget. */
        LOGI(TAG, "PUB %s  %u bytes  ok", TOPIC_TELEMETRY, (unsigned)n);
        LOGI(TAG, "    %s", buf);
#endif
    } else {
        /* PubSubClient returns false both for a transport failure and for a
         * payload larger than its buffer, so print the size next to the state -
         * a 900 byte payload against a 256 byte buffer looks like a network
         * fault until you see the two numbers together. */
        LOGW(TAG, "telemetry publish FAILED: %s (%d), payload %u bytes, buffer %u",
             mqttStateName(gMqtt.state()), gMqtt.state(), (unsigned)n,
             (unsigned)MQTT_BUFFER_BYTES);
    }
}

/**
 * Matches "<prefix>N" with N in 1..4 and nothing after it, yielding N-1.
 *
 * The trailing-NUL check is what keeps "getRelays" out of the per-relay branch:
 * it shares the first eight characters with "getRelay1" and differs only in
 * what follows, and the two answer in different shapes.
 */
static bool isRelayIndexMethod(const char* method, const char* prefix, uint8_t* indexOut) {
    size_t n = strlen(prefix);
    if (strncmp(method, prefix, n) != 0) return false;
    if (method[n] < '1' || method[n] > '4' || method[n + 1] != '\0') return false;
    *indexOut = (uint8_t)(method[n] - '1');
    return true;
}

/**
 * Makes sure the cached LOCAL_IO snapshot is younger than `maxAgeMs`.
 *
 * Returns false when there is nothing trustworthy to answer from, which the
 * callers turn into the de-energized reading rather than into an error object.
 */
bool TbClient::ensureFresh(uint32_t maxAgeMs) {
    if (_io == nullptr || !_io->isReady()) return false;

    LocalIoState s = _io->state();
    if (s.valid && (uint32_t)(millis() - s.sampledMs) <= maxAgeMs) return true;

    return _io->refresh();
}

bool TbClient::relayStateOrFalse(uint8_t index) const {
    if (_io == nullptr || !_io->isReady()) return false;
    if (!_io->state().valid) return false;
    return _io->relay(index);
}

uint8_t TbClient::relayMaskOrZero() const {
    if (_io == nullptr || !_io->isReady()) return 0u;
    LocalIoState s = _io->state();
    return s.valid ? s.relayMask : 0u;
}

/**
 * Answers one RPC.
 *
 * TWO RULES, AND THE SECOND ONE IS EASY TO GET WRONG
 *
 * 1. Every `set` re-reads the pins before replying, and reports what it found
 *    rather than what it asked for. Confirming a command by echoing it back is
 *    the single easiest way to build a dashboard that looks healthy while the
 *    plant is not.
 *
 * 2. Anything a control widget reads answers with a BARE JSON value - `true`,
 *    `false`, `7` - never wrapped in an object. ThingsBoard's switch widget
 *    coerces the whole response body to a boolean, and every object is truthy,
 *    so `{"result": false}` renders as ON. See the block comment in
 *    tb_client.h for why that only ever showed up on a page refresh.
 *
 *    The failure paths obey the same rule: they answer `false` / `0`, not an
 *    error object, and put the reason in the log.
 */
void TbClient::handleRpc(const char* requestId, const uint8_t* payload, unsigned int len) {
    JsonDocument req;
    DeserializationError err = deserializeJson(req, payload, len);

    char topic[64];
    snprintf(topic, sizeof(topic), "v1/devices/me/rpc/response/%s", requestId);

    JsonDocument rsp;

    if (err) {
        rsp["error"] = "bad json";
        LOGW(TAG, "RPC %s: bad json (%s)", requestId, err.c_str());
    } else {
        const char* method = req["method"] | "";
        JsonVariant params = req["params"];
        ++_rpcCount;

        LOGI(TAG, "RPC %s method=%s", requestId, method);

        /* The method is classified before any work is attempted, because the
         * failure paths have to answer in the same shape as the success paths.
         * A widget that asked for a boolean and got an object does not report
         * an error - it renders ON. */
        uint8_t index = 0;
        bool isSetRelayN = isRelayIndexMethod(method, "setRelay", &index);
        bool isGetRelayN = !isSetRelayN && isRelayIndexMethod(method, "getRelay", &index);
        bool wantsBool = isSetRelayN || isGetRelayN;
        bool wantsNumber = (strcmp(method, "setRelays") == 0 || strcmp(method, "getRelays") == 0 ||
                            strcmp(method, "getInputs") == 0);

        if (_io == nullptr || !_io->isReady()) {
            LOGW(TAG, "RPC %s: local IO not available", method);
            if (wantsBool) {
                rsp.set(false);
            } else if (wantsNumber) {
                rsp.set(0);
            } else {
                rsp["error"] = "local IO not available";
            }

        } else if (isSetRelayN) {
            /* A ThingsBoard switch widget sends a bare boolean; a button may
             * send a number or a string. Accepting all three costs three lines
             * and removes a class of "the widget does nothing" reports that are
             * really a type mismatch. */
            bool on;
            if (params.is<bool>()) {
                on = params.as<bool>();
            } else if (params.is<int>()) {
                on = params.as<int>() != 0;
            } else if (params.is<const char*>()) {
                const char* s = params.as<const char*>();
                on = (strcmp(s, "true") == 0 || strcmp(s, "1") == 0 || strcmp(s, "on") == 0);
            } else if (params.is<JsonObject>() && !params["value"].isNull()) {
                on = params["value"].as<bool>();
            } else {
                on = false;
            }

            StmV2Status st = _io->setRelay(index, on);
            if (st != V2_OK) {
                LOGW(TAG, "RPC %s refused: %s (dev %s)", method, stmV2StatusName(st),
                     gwStatusName(_io->lastDeviceStatus()));
            }

            /* Refused or accepted, the answer is what the pins say. A refused
             * command must leave the switch sitting where the relay really is,
             * not bounce back to where the operator dragged it. */
            _io->refresh();
            rsp.set(relayStateOrFalse(index));

        } else if (isGetRelayN) {
            /* This is the branch a page refresh lands in: the widget has no
             * memory of its position and asks the device for it. */
            ensureFresh(RPC_MAX_AGE_MS);
            rsp.set(relayStateOrFalse(index));

        } else if (strcmp(method, "setRelays") == 0) {
            long mask = params.is<int>() ? params.as<long>() : -1;
            if (mask < 0 || mask > 0x0F) {
                LOGW(TAG, "RPC setRelays: params must be 0..15");
            } else {
                StmV2Status st = _io->setRelayMask((uint8_t)mask);
                if (st != V2_OK) {
                    LOGW(TAG, "RPC setRelays refused: %s (dev %s)", stmV2StatusName(st),
                         gwStatusName(_io->lastDeviceStatus()));
                }
                _io->refresh();
            }
            rsp.set(relayMaskOrZero());

        } else if (strcmp(method, "getRelays") == 0) {
            ensureFresh(RPC_MAX_AGE_MS);
            rsp.set(relayMaskOrZero());

        } else if (strcmp(method, "getInputs") == 0) {
            ensureFresh(RPC_MAX_AGE_MS);
            LocalIoState s = _io->state();
            rsp.set(s.valid ? s.inputMask : 0u);

        } else if (strcmp(method, "getStatus") == 0) {
            ensureFresh(RPC_MAX_AGE_MS);
            LocalIoState s = _io->state();
            StmV2Stats ls = stmLinkV2.stats();
            JsonObject r = rsp["result"].to<JsonObject>();
            r["relayMask"] = s.relayMask;
            r["inputMask"] = s.inputMask;
            r["quality"] = gwQualityName(s.quality);
            r["valid"] = s.valid;
            r["polls"] = _io->pollCount();
            r["pollErrors"] = _io->errorCount();
            r["linkRttUs"] = ls.lastRttUs;
            r["linkTimeouts"] = ls.timeouts;
            r["rssi"] = WiFi.RSSI();
            r["uptimeSec"] = millis() / 1000UL;

        } else {
            rsp["error"] = "unknown method";
            LOGW(TAG, "RPC unknown method '%s'", method);
        }
    }

    char buf[512];
    size_t n = serializeJson(rsp, buf, sizeof(buf));
    if (n == 0 || n >= sizeof(buf)) {
        LOGE(TAG, "RPC response did not fit");
        return;
    }
    gMqtt.publish(topic, buf, false);

    /* An RPC that changed something publishes straight away rather than waiting
     * for the next scheduled snapshot, so every other dashboard watching this
     * device sees the new state at the same moment the caller does. */
    publishTelemetry();
}

void TbClient::onMessage(const char* topic, const uint8_t* payload, unsigned int len) {
    size_t prefixLen = strlen(TOPIC_RPC_PREFIX);

    if (strncmp(topic, TOPIC_RPC_PREFIX, prefixLen) == 0) {
        handleRpc(topic + prefixLen, payload, len);
        return;
    }

    LOGD(TAG, "message on unhandled topic %s (%u bytes)", topic, len);
}

void TbClient::loop() {
    if (!deviceConfig.isProvisioned()) return;

    if (!ensureWifi()) return;
    if (!ensureMqtt()) return;

    gMqtt.loop();

    uint32_t now = millis();
    bool due = (int32_t)(now - _nextTelemetryMs) >= 0;

    /* Change-driven publishing on top of the schedule. An input closing has to
     * reach the dashboard now, not up to telemetryMs later; a machine that
     * simply is not moving still has to keep reporting, or it cannot be told
     * apart from a dead gateway. */
    bool changed = false;
    if (_io && _io->isReady() && _io->state().valid) {
        LocalIoState s = _io->state();
        changed = !_everPublished || s.relayMask != _lastRelayMask ||
                  s.inputMask != _lastInputMask;
    }

    if (due || changed) {
        uint32_t period = deviceConfig.config().telemetryMs;
        if (period == 0) period = TELEMETRY_INTERVAL_MS;
        _nextTelemetryMs = now + period;
        publishTelemetry();
    }

    if (!_attributesSent && (int32_t)(now - _nextAttributesMs) >= 0) {
        publishAttributes();
    }

#if CLOUD_TRACE
    /* Says what the uplink is doing even when nothing is changing, so a quiet
     * console can be told apart from a stuck one. */
    if ((int32_t)(now - _nextTraceMs) >= 0) {
        _nextTraceMs = now + CLOUD_TRACE_STATUS_MS;
        LOGI(TAG, "cloud: wifi=%s ip=%s rssi=%d mqtt=%s publishes=%lu rpc=%lu ioReady=%d",
             WiFi.status() == WL_CONNECTED ? "up" : "down",
             WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(),
             mqttStateName(gMqtt.state()), (unsigned long)_publishCount,
             (unsigned long)_rpcCount, (int)(_io != nullptr && _io->isReady()));
    }
#endif
}

bool TbClient::isConnected() const { return gMqtt.connected(); }

void TbClient::reportStatus() const {
    char buf[32];

    provEmit("cloud.backend", "thingsboard");
    provEmit("cloud.connected", gMqtt.connected() ? "1" : "0");
    snprintf(buf, sizeof(buf), "%d (%s)", gMqtt.state(), mqttStateName(gMqtt.state()));
    provEmit("cloud.mqttState", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)_publishCount);
    provEmit("cloud.publishes", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)_reconnectCount);
    provEmit("cloud.reconnects", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)_rpcCount);
    provEmit("cloud.rpcCalls", buf);
}
