/**
 * provisioning.cpp - line console on UART0. See provisioning.h for the protocol.
 */
#include "provisioning.h"

#include <WiFi.h>
#include <string.h>

#include "config.h"
#include "device_config.h"
#include "log.h"

static const char* TAG = "prov";

#define PROV_LINE_MAX 200

static char gLine[PROV_LINE_MAX];
static uint16_t gLen;
static bool gOverflow;
static ProvStatusHook gStatusHook;

void provEmit(const char* key, const char* value) {
    Serial.printf("+%s=%s\r\n", key, value ? value : "");
}

static void emitOk() { Serial.print("+OK\r\n"); }

static void emitErr(const char* why) { Serial.printf("+ERR %s\r\n", why); }

/** Case-insensitive compare of the command word against a literal. */
static bool wordIs(const char* s, size_t len, const char* lit) {
    if (strlen(lit) != len) return false;
    for (size_t i = 0; i < len; ++i) {
        if (toupper((unsigned char)s[i]) != toupper((unsigned char)lit[i])) return false;
    }
    return true;
}

static void cmdIdentify() {
    char buf[96];
    const DeviceConfig& c = deviceConfig.config();

    snprintf(buf, sizeof(buf), "name=%s fw=%s mac=%s provisioned=%d", c.deviceName,
             GW_FW_STRING, WiFi.macAddress().c_str(), (int)deviceConfig.isProvisioned());
    Serial.printf("+GW %s\r\n", buf);
    emitOk();
}

static void cmdGet() {
    deviceConfig.forEachField(provEmit);
    provEmit("provisioned", deviceConfig.isProvisioned() ? "1" : "0");
    emitOk();
}

static void cmdSet(char* args) {
    char* key = args;
    char* value;

    while (*key == ' ') ++key;
    value = strchr(key, ' ');
    if (value == nullptr) {
        /* A key with no value clears the field. Needed for a real case: moving
         * a unit from a WPA2 network to an open one has to be able to remove
         * the stored passphrase, and "SET wifi.pass" with nothing after it is
         * the natural way to say that. */
        if (*key == '\0') {
            emitErr("usage: SET <key> <value>");
            return;
        }
        if (!deviceConfig.setByKey(key, "")) {
            emitErr("unknown key");
            return;
        }
        emitOk();
        return;
    }

    *value++ = '\0';
    /* Only the separating space is consumed. Everything after it is the value
     * verbatim, including further spaces - WiFi passphrases contain them. */
    if (!deviceConfig.setByKey(key, value)) {
        emitErr("unknown key or value out of range");
        return;
    }
    emitOk();
}

static void cmdStatus() {
    char buf[64];

    snprintf(buf, sizeof(buf), "%d", (int)WiFi.status());
    provEmit("wifi.status", buf);
    provEmit("wifi.ip", WiFi.localIP().toString().c_str());
    snprintf(buf, sizeof(buf), "%d", (int)WiFi.RSSI());
    provEmit("wifi.rssi", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(millis() / 1000UL));
    provEmit("sys.uptimeSec", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)ESP.getFreeHeap());
    provEmit("sys.freeHeap", buf);

    /* Whatever the firmware that owns this build wants to add. */
    if (gStatusHook) gStatusHook();

    emitOk();
}

static void handleLine(char* line) {
    char* cmd = line;
    char* args;
    size_t cmdLen;

    while (*cmd == ' ' || *cmd == '\t') ++cmd;
    if (*cmd == '\0') return; /* a bare newline is not an error */

    args = strchr(cmd, ' ');
    if (args != nullptr) {
        cmdLen = (size_t)(args - cmd);
        ++args;
    } else {
        cmdLen = strlen(cmd);
        args = cmd + cmdLen; /* points at the terminator - an empty argument */
    }

    if (wordIs(cmd, cmdLen, "GW?")) {
        cmdIdentify();
    } else if (wordIs(cmd, cmdLen, "GET")) {
        cmdGet();
    } else if (wordIs(cmd, cmdLen, "SET")) {
        cmdSet(args);
    } else if (wordIs(cmd, cmdLen, "SAVE")) {
        if (deviceConfig.save()) {
            emitOk();
        } else {
            emitErr("nvs write failed");
        }
    } else if (wordIs(cmd, cmdLen, "CLEAR")) {
        if (deviceConfig.clear()) {
            emitOk();
        } else {
            emitErr("nvs erase failed");
        }
    } else if (wordIs(cmd, cmdLen, "STATUS")) {
        cmdStatus();
    } else if (wordIs(cmd, cmdLen, "REBOOT")) {
        emitOk();
        Serial.flush();
        /* Let the reply actually leave the UART. ESP.restart() is immediate and
         * a 115200 baud line still holding six characters would lose them, so
         * the tool would see a reboot it could not confirm. */
        delay(80);
        ESP.restart();
    } else {
        emitErr("unknown command");
    }
}

void provisioningBegin(ProvStatusHook statusHook) {
    gStatusHook = statusHook;
    gLen = 0;
    gOverflow = false;
    LOGI(TAG, "console ready - GW? GET SET SAVE CLEAR STATUS REBOOT");
}

void provisioningPoll() {
    while (Serial.available() > 0) {
        int ch = Serial.read();
        if (ch < 0) break;

        if (ch == '\r') continue;

        if (ch == '\n') {
            if (gOverflow) {
                /* Report the overflow rather than acting on a truncated
                 * command. Executing the first 200 characters of a longer line
                 * would set a field to a silently shortened value. */
                emitErr("line too long");
                gOverflow = false;
                gLen = 0;
                continue;
            }
            gLine[gLen] = '\0';
            handleLine(gLine);
            gLen = 0;
            continue;
        }

        if (gLen < (PROV_LINE_MAX - 1)) {
            gLine[gLen++] = (char)ch;
        } else {
            gOverflow = true;
        }
    }
}
