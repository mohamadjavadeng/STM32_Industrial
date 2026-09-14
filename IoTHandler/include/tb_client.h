/**
 * tb_client.h - WiFi + MQTT to ThingsBoard, authenticated by device token.
 * ---------------------------------------------------------------------------
 *
 * The sibling of cloud_client.h, which speaks AWS IoT Core with X.509 client
 * certificates. Same job, different back end, and deliberately a separate class
 * rather than a mode flag inside one: the two differ in how they authenticate,
 * how topics are shaped, and how a downlink arrives. Threading all three
 * differences through one class with conditionals produces code where neither
 * path is readable and a change to one silently affects the other.
 *
 * WHY THINGSBOARD IS THE EASY ONE TO COMMISSION
 *   Authentication is a single access token, short enough to paste into a
 *   dialog. That is what lets the whole configuration live in NVS and be
 *   written over USB by tools/gw_config_tool.py, with no certificates to
 *   generate, no clock to be correct before the handshake, and one firmware
 *   binary for every unit.
 *
 * TOPICS (fixed by ThingsBoard, not by us - the token identifies the device)
 *   v1/devices/me/telemetry            pub   timeseries
 *   v1/devices/me/attributes           pub   device attributes
 *   v1/devices/me/rpc/request/+        sub   server-to-device RPC
 *   v1/devices/me/rpc/response/{id}    pub   our answer to one RPC
 *
 * THE RPC METHODS THE DASHBOARD CALLS
 *   setRelay1..setRelay4   params: true / false / 0 / 1
 *   getRelay1..getRelay4   returns the relay's ACTUAL state
 *   setRelays / getRelays  all four as a 0..15 bitmask
 *   getInputs              the four inputs as a 0..15 bitmask
 *   getStatus              a small object of link and IO diagnostics
 *
 *   Every get answers from the last poll of the STM32, and every set is
 *   confirmed by re-reading the pins before it replies. A control dashboard
 *   that echoes the command back as if it were the state is lying to the
 *   operator at exactly the moment the wire has come loose.
 *
 * PUBLISH POLICY
 *   A full snapshot every telemetryMs, plus an immediate publish whenever a
 *   relay or an input changes. Periodic alone would put up to ten seconds
 *   between a limit switch closing and the dashboard showing it, which is
 *   useless for anything being watched; change-driven alone would leave a quiet
 *   machine looking disconnected, with no way to tell a steady input from a
 *   dead gateway.
 */
#pragma once

#include <Arduino.h>

#include "config.h"
#include "localio_client.h"

class TbClient {
   public:
    TbClient();

    /**
     * Wires in the IO cache the publisher reads from. Does not connect - the
     * first connection attempt happens in loop(), so a missing access point
     * never blocks startup.
     */
    bool begin(LocalIoClient* io);

    /** Drives WiFi, MQTT, the publish schedule and inbound RPC. Never blocks long. */
    void loop();

    bool isConnected() const;

    /** Publishes a snapshot now, whatever the schedule says. */
    void publishTelemetry();

    /** Emits `cloud.*` lines for the provisioning console's STATUS command. */
    void reportStatus() const;

    uint32_t publishCount() const { return _publishCount; }
    uint32_t reconnectCount() const { return _reconnectCount; }
    uint32_t rpcCount() const { return _rpcCount; }

   private:
    bool ensureWifi();
    bool ensureMqtt();
    void publishAttributes();
    void handleRpc(const char* requestId, const uint8_t* payload, unsigned int len);
    void onMessage(const char* topic, const uint8_t* payload, unsigned int len);

    static void trampoline(char* topic, uint8_t* payload, unsigned int len);
    static TbClient* _self;

    LocalIoClient* _io;

    uint32_t _nextTelemetryMs;
    uint32_t _nextConnectMs;
    uint32_t _nextAttributesMs; // throttles the attribute retry
    uint32_t _nextTraceMs;      // throttles the periodic connection line
    uint32_t _nextSkipLogMs;    // throttles "skipped, not connected"
    uint32_t _backoffMs;
    uint32_t _wifiStartedMs;
    uint32_t _publishCount;
    uint32_t _reconnectCount;
    uint32_t _rpcCount;

    // Last published IO, so a change can be detected without re-reading.
    uint8_t _lastRelayMask;
    uint8_t _lastInputMask;
    bool _everPublished;
    bool _attributesSent;
};

extern TbClient tbClient;
