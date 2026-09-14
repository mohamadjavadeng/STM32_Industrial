/**
 * device_config.h - the settings a commissioning engineer owns, kept in NVS.
 * ---------------------------------------------------------------------------
 *
 * WHY THIS EXISTS ALONGSIDE secrets.h
 *   secrets.h is a build-time header: changing a WiFi password means a
 *   toolchain, a rebuild and a flash. That is fine for one bench unit and
 *   impossible for a panel already screwed into a cabinet, where the person who
 *   knows the site WiFi is not the person who owns the source tree.
 *
 *   Everything here lives in NVS instead, written over USB serial by
 *   tools/gw_config_tool.py. The firmware binary becomes identical for every
 *   unit, and identity arrives at commissioning time - which is also what makes
 *   the same image safe to hand to a contractor.
 *
 *   secrets.h is untouched and still owns the AWS X.509 build. ThingsBoard
 *   authenticates with a single device access token, which is short enough to
 *   type into a dialog box and therefore belongs in NVS rather than in flash.
 *
 * WHAT IS DELIBERATELY NOT HERE
 *   The tag map, the slot numbering and anything else that both chips have to
 *   agree on. Those come from the STM32 at runtime (RD_META) or from the shared
 *   protocol header. A field-editable copy of a two-sided contract is a way to
 *   get the two sides out of step with nothing to detect it.
 *
 * STORAGE
 *   One NVS key per field, not one blob. A blob makes adding a field a
 *   migration: old units come back with a short read, and the usual outcome is
 *   a struct that silently reads garbage past the end. Per-key means an unknown
 *   key reads as its default and a new firmware inherits every setting that
 *   already existed.
 */
#pragma once

#include <Arduino.h>

/* Field widths. WiFi's own limits set the first two: SSID is at most 32 bytes,
 * a WPA2 passphrase at most 63, both plus a terminator. The token width is
 * generous - a ThingsBoard access token is 20 characters today. */
#define CFG_SSID_LEN 33
#define CFG_PASS_LEN 64
#define CFG_HOST_LEN 64
#define CFG_TOKEN_LEN 48
#define CFG_NAME_LEN 32

struct DeviceConfig {
    char wifiSsid[CFG_SSID_LEN];
    char wifiPass[CFG_PASS_LEN];
    char tbHost[CFG_HOST_LEN];   // demo.thingsboard.io, or your own server
    uint16_t tbPort;             // 1883 plain, 8883 TLS
    char tbToken[CFG_TOKEN_LEN]; // device access token = MQTT username
    char deviceName[CFG_NAME_LEN];
    bool tbUseTls;
    uint32_t telemetryMs; // publish period; 0 = fall back to the build default
};

class DeviceConfigStore {
   public:
    DeviceConfigStore();

    /**
     * Reads NVS into the in-memory copy, filling anything absent with a
     * default. Always succeeds: a unit with empty NVS comes up unprovisioned
     * and says so, rather than refusing to boot.
     */
    bool load();

    /** Writes the in-memory copy back to NVS. False if the NVS write failed. */
    bool save();

    /** Erases the namespace. The next boot comes up unprovisioned. */
    bool clear();

    /**
     * True when there is enough here to try: an SSID, a host and a token.
     *
     * The password is deliberately not part of the test, because an open guest
     * network is a legitimate configuration and demanding a password for one
     * would make a valid setup look broken.
     */
    bool isProvisioned() const;

    DeviceConfig& mutableConfig() { return _cfg; }
    const DeviceConfig& config() const { return _cfg; }

    /**
     * Sets one field by the same key name the serial console and the GUI tool
     * use ("wifi.ssid", "tb.token", ...). Returns false for an unknown key or a
     * value that does not fit, so a typo in the tool is reported rather than
     * silently dropped.
     *
     * Central on purpose: the console, a future captive portal and anything
     * else that sets configuration all go through this one validator, so none
     * of them can invent its own idea of what a valid host is.
     */
    bool setByKey(const char* key, const char* value);

    /**
     * Prints every field as `key=value` lines through `emit`.
     *
     * The password is emitted as its length only, never its characters. The
     * tool needs to know whether one is stored - an empty field means an open
     * network - and nothing needs to read it back. A console that echoes the
     * site WiFi password to anyone with a USB cable is a worse problem than the
     * inconvenience of retyping it.
     */
    void forEachField(void (*emit)(const char* key, const char* value)) const;

   private:
    DeviceConfig _cfg;
};

extern DeviceConfigStore deviceConfig;
