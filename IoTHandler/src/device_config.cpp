/**
 * device_config.cpp - NVS-backed runtime configuration.
 *
 * See device_config.h for why this exists next to secrets.h.
 */
#include "device_config.h"

#include <Preferences.h>
#include <string.h>

#include "config.h"
#include "log.h"

static const char* TAG = "cfg";

/* NVS namespace. Short because NVS caps a namespace at 15 characters, and
 * distinct from anything the Arduino core uses for itself. */
static const char* NS = "gwcfg";

/* Key names, used identically in NVS, on the serial console and in the GUI
 * tool. One spelling everywhere means a field added here needs no matching
 * change in the tool's protocol - the tool lists what the device reports. */
static const char* K_SSID = "wifi.ssid";
static const char* K_PASS = "wifi.pass";
static const char* K_HOST = "tb.host";
static const char* K_PORT = "tb.port";
static const char* K_TOKEN = "tb.token";
static const char* K_NAME = "dev.name";
static const char* K_TLS = "tb.tls";
static const char* K_TELE = "tb.telemetryMs";

DeviceConfigStore deviceConfig;

/** strncpy that always terminates and never warns about truncation. */
static void copyField(char* dst, size_t cap, const char* src) {
    if (cap == 0) return;
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

DeviceConfigStore::DeviceConfigStore() {
    memset(&_cfg, 0, sizeof(_cfg));
    _cfg.tbPort = 1883;
    _cfg.tbUseTls = false;
    _cfg.telemetryMs = TELEMETRY_INTERVAL_MS;
    copyField(_cfg.tbHost, CFG_HOST_LEN, "demo.thingsboard.io");
    copyField(_cfg.deviceName, CFG_NAME_LEN, "IoTPLC");
}

bool DeviceConfigStore::load() {
    Preferences p;

    /* Read-only open. If the namespace does not exist yet - a factory-fresh
     * unit - this returns false and every default set in the constructor
     * stands. That is a normal first boot, not an error. */
    if (!p.begin(NS, true)) {
        LOGW(TAG, "no stored config, using defaults");
        return false;
    }

    String s;
    s = p.getString(K_SSID, _cfg.wifiSsid);
    copyField(_cfg.wifiSsid, CFG_SSID_LEN, s.c_str());
    s = p.getString(K_PASS, _cfg.wifiPass);
    copyField(_cfg.wifiPass, CFG_PASS_LEN, s.c_str());
    s = p.getString(K_HOST, _cfg.tbHost);
    copyField(_cfg.tbHost, CFG_HOST_LEN, s.c_str());
    s = p.getString(K_TOKEN, _cfg.tbToken);
    copyField(_cfg.tbToken, CFG_TOKEN_LEN, s.c_str());
    s = p.getString(K_NAME, _cfg.deviceName);
    copyField(_cfg.deviceName, CFG_NAME_LEN, s.c_str());

    _cfg.tbPort = p.getUShort(K_PORT, _cfg.tbPort);
    _cfg.tbUseTls = p.getBool(K_TLS, _cfg.tbUseTls);
    _cfg.telemetryMs = p.getULong(K_TELE, _cfg.telemetryMs);

    p.end();

    LOGI(TAG, "loaded: ssid='%s' host=%s:%u tls=%d token=%s name=%s", _cfg.wifiSsid, _cfg.tbHost,
         (unsigned)_cfg.tbPort, (int)_cfg.tbUseTls, _cfg.tbToken[0] ? "set" : "MISSING",
         _cfg.deviceName);
    return true;
}

bool DeviceConfigStore::save() {
    Preferences p;

    if (!p.begin(NS, false)) {
        LOGE(TAG, "NVS open for write failed");
        return false;
    }

    bool ok = true;
    ok &= p.putString(K_SSID, _cfg.wifiSsid) > 0 || _cfg.wifiSsid[0] == '\0';
    ok &= p.putString(K_PASS, _cfg.wifiPass) > 0 || _cfg.wifiPass[0] == '\0';
    ok &= p.putString(K_HOST, _cfg.tbHost) > 0 || _cfg.tbHost[0] == '\0';
    ok &= p.putString(K_TOKEN, _cfg.tbToken) > 0 || _cfg.tbToken[0] == '\0';
    ok &= p.putString(K_NAME, _cfg.deviceName) > 0 || _cfg.deviceName[0] == '\0';
    ok &= p.putUShort(K_PORT, _cfg.tbPort) > 0;
    ok &= p.putBool(K_TLS, _cfg.tbUseTls) > 0;
    ok &= p.putULong(K_TELE, _cfg.telemetryMs) > 0;

    p.end();

    if (ok) {
        LOGI(TAG, "config saved");
    } else {
        LOGE(TAG, "config save incomplete");
    }
    return ok;
}

bool DeviceConfigStore::clear() {
    Preferences p;
    if (!p.begin(NS, false)) return false;
    bool ok = p.clear();
    p.end();
    LOGW(TAG, "config cleared: %d", (int)ok);
    return ok;
}

bool DeviceConfigStore::isProvisioned() const {
    return _cfg.wifiSsid[0] != '\0' && _cfg.tbHost[0] != '\0' && _cfg.tbToken[0] != '\0';
}

bool DeviceConfigStore::setByKey(const char* key, const char* value) {
    if (key == nullptr || value == nullptr) return false;

    /* Length is checked before the copy, and an over-long value is refused
     * rather than truncated. A token quietly cut to 47 characters would fail
     * authentication with an error that points at the broker, not at the field
     * that was too long. */
    if (strcmp(key, K_SSID) == 0) {
        if (strlen(value) >= CFG_SSID_LEN) return false;
        copyField(_cfg.wifiSsid, CFG_SSID_LEN, value);
        return true;
    }
    if (strcmp(key, K_PASS) == 0) {
        if (strlen(value) >= CFG_PASS_LEN) return false;
        copyField(_cfg.wifiPass, CFG_PASS_LEN, value);
        return true;
    }
    if (strcmp(key, K_HOST) == 0) {
        if (strlen(value) >= CFG_HOST_LEN) return false;
        copyField(_cfg.tbHost, CFG_HOST_LEN, value);
        return true;
    }
    if (strcmp(key, K_TOKEN) == 0) {
        if (strlen(value) >= CFG_TOKEN_LEN) return false;
        copyField(_cfg.tbToken, CFG_TOKEN_LEN, value);
        return true;
    }
    if (strcmp(key, K_NAME) == 0) {
        if (strlen(value) >= CFG_NAME_LEN) return false;
        copyField(_cfg.deviceName, CFG_NAME_LEN, value);
        return true;
    }
    if (strcmp(key, K_PORT) == 0) {
        long v = strtol(value, nullptr, 10);
        if (v < 1 || v > 65535) return false;
        _cfg.tbPort = (uint16_t)v;
        return true;
    }
    if (strcmp(key, K_TLS) == 0) {
        _cfg.tbUseTls = (value[0] == '1' || value[0] == 't' || value[0] == 'T');
        return true;
    }
    if (strcmp(key, K_TELE) == 0) {
        long v = strtol(value, nullptr, 10);
        /* Below a second the gateway spends its time publishing rather than
         * polling, and ThingsBoard rate-limits a device that does. */
        if (v < 1000 || v > 3600000L) return false;
        _cfg.telemetryMs = (uint32_t)v;
        return true;
    }
    return false;
}

void DeviceConfigStore::forEachField(void (*emit)(const char* key, const char* value)) const {
    char buf[16];

    emit(K_SSID, _cfg.wifiSsid);

    /* The password never leaves the device. Its length is enough for the tool
     * to show "set" versus "not set", which is the only thing anyone needs to
     * know, and an empty field legitimately means an open network. */
    snprintf(buf, sizeof(buf), "<%u chars>", (unsigned)strlen(_cfg.wifiPass));
    emit(K_PASS, strlen(_cfg.wifiPass) ? buf : "");

    emit(K_HOST, _cfg.tbHost);
    snprintf(buf, sizeof(buf), "%u", (unsigned)_cfg.tbPort);
    emit(K_PORT, buf);
    emit(K_TOKEN, _cfg.tbToken);
    emit(K_NAME, _cfg.deviceName);
    emit(K_TLS, _cfg.tbUseTls ? "1" : "0");
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)_cfg.telemetryMs);
    emit(K_TELE, buf);
}
