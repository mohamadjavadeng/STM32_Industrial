/**
 * cloud_client.h - WiFi + SNTP + MQTT/TLS to AWS IoT Core.
 *
 * Runs entirely on the network task. Never touches the STM32 link directly:
 * uplink data comes from a ProcessImage snapshot, downlink writes go out through
 * the command queue. Compiled out completely when ENABLE_CLOUD=0 so the serial
 * link can be brought up before any certificates exist.
 *
 * Topics, all under DEVICE_ID:
 *   <id>/telemetry   pub   periodic snapshot of every tag
 *   <id>/status      pub   connectivity + link diagnostics, and the LWT
 *   <id>/cmd         sub   write requests
 *   <id>/cmd/ack     pub   result of each write request
 */
#pragma once

#include <Arduino.h>

#include "commands.h"
#include "config.h"

class CloudClient {
   public:
    CloudClient();

    /** Stores the queues used to hand work to, and collect it from, the link task. */
    bool begin(QueueHandle_t cmdQueue, QueueHandle_t ackQueue);

    /** Drives WiFi, TLS, MQTT and the publish schedule. Call often, never blocks long. */
    void loop();

    bool isConnected() const;
    uint32_t publishCount() const { return _publishCount; }
    uint32_t reconnectCount() const { return _reconnectCount; }

   private:
    bool ensureWifi();
    bool ensureTime();
    bool ensureMqtt();
    void publishTelemetry();
    void publishStatus(bool online);
    void drainAcks();
    void onMessage(const char* topic, const uint8_t* payload, unsigned int len);

    static void trampoline(char* topic, uint8_t* payload, unsigned int len);
    static CloudClient* _self;

    QueueHandle_t _cmdQueue;
    QueueHandle_t _ackQueue;

    bool _timeValid;
    uint32_t _nextTelemetryMs;
    uint32_t _nextStatusMs;
    uint32_t _nextConnectMs;
    uint32_t _backoffMs;
    uint32_t _publishCount;
    uint32_t _reconnectCount;
    uint32_t _seq;
    bool _announced;

    char _topicTelemetry[96];
    char _topicStatus[96];
    char _topicCmd[96];
    char _topicAck[96];
};

extern CloudClient cloudClient;
