/**
 * config.h - build-time configuration for the ESP32-S3 network half.
 *
 * Everything that depends on the board wiring, the STM32 firmware revision or
 * the cloud deployment lives here. No secrets - those go in secrets.h.
 */
#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Serial link to the STM32
// ---------------------------------------------------------------------------
// STM32 side is USART3: PB10 = USART3_TX, PB11 = USART3_RX, 115200 8N1.
// Cross them: ESP RX <- PB10, ESP TX -> PB11. Common ground is mandatory.
//
// Pins avoid: 0/45/46 (strapping), 19/20 (native USB), 26..37 (octal flash +
// PSRAM on the N8R8 module), 43/44 (UART0 debug console), 48 (RGB LED).
#define PIN_STM_RX 18  // ESP32 RX  <-- STM32 PB10 (USART3_TX)
#define PIN_STM_TX 17  // ESP32 TX  --> STM32 PB11 (USART3_RX)

// Optional side-band signals. Set to -1 when not wired.
// PIN_STM_EVENT: STM32 -> ESP32, "I have something for you, poll me now".
//                Lets alarms bypass the poll interval while keeping a strict
//                single-initiator protocol. Not driven by STM32 V1.2 yet.
// PIN_STM_RESET: ESP32 -> STM32 NRST, active low, drive as open-drain.
//                Leave -1 unless you actually want the radio side able to
//                reset the control side (usually you want the opposite).
#define PIN_STM_EVENT -1
#define PIN_STM_RESET -1

// Must match huart3.Init.BaudRate in ../ModbusRTU_AWS/Core/Src/main.c
#define STM_LINK_BAUD 115200UL

// The STM32 handler answers an ESP32 request by running a *blocking* Modbus
// transaction on RS-485 (HAL_MAX_DELAY transmit + 1000 ms receive timeout in
// Modbus_SendRequest). So our per-request timeout must exceed 1000 ms or we
// will time out on every slow/absent field device.
#define STM_RSP_TIMEOUT_MS 1600
#define STM_BYTE_TIMEOUT_MS 60    // gap tolerance once a frame has started
#define STM_MAX_ATTEMPTS 3        // total tries per transaction
#define STM_RETRY_BASE_MS 40      // backoff = base * attempt
#define STM_GAP_MS 4              // quiet time between transactions
#define STM_RESYNC_QUIET_MS 120   // RX must be idle this long after a resync
#define STM_RESYNC_CAP_MS 500     // hard cap on a resync
#define STM_RX_BUFFER 1024        // >= largest response (2*50+5 = 205 bytes)

// 1 = target the STM32 firmware as it exists today (V1.2). Enables the
//     work-arounds documented in stm_protocol.h: the swapped HOLDING/INPUT
//     register type on single reads, byte-vs-register count on block reads,
//     and the parser resync after a failed block read.
// 0 = target a fixed STM32 firmware where type codes mean what they say.
#ifndef STM_FW_QUIRKS
#define STM_FW_QUIRKS 1
#endif

// ---------------------------------------------------------------------------
// Link protocol v2 (0xA5 / 0x5A) - stm_link_v2.h
// ---------------------------------------------------------------------------
// v2 talks to a process image instead of proxying a Modbus transaction, so the
// STM32 answers in microseconds rather than in however long the slowest RS-485
// slave takes. Everything below is an order of magnitude tighter than the v1
// numbers above, and that is the point of the rewrite - not the framing.
//
// The STM32 side must be built with GW_LINK_ENABLE 1 (ModbusRTU_AWS/Core/Inc/
// gw_link_cfg.h). Only one protocol can own USART3.

// 100 ms is ~40x the worst observed bench round trip and still 16x tighter than
// v1's 1600 ms. Generous because the STM32 superloop will grow; tighten it once
// the scan cycle is measured and stable.
#define STMV2_RSP_TIMEOUT_MS 100
// Gap tolerance once a frame has started arriving. One byte is 87 us at
// 115200 baud, so 20 ms is 200 byte-times of slack for an interrupt storm.
#define STMV2_BYTE_TIMEOUT_MS 20
#define STMV2_MAX_ATTEMPTS 3    // total tries per transaction
#define STMV2_RETRY_BASE_MS 10  // backoff = base * attempt number
#define STMV2_GAP_MS 2          // quiet time before a retry goes out

// Must hold the largest reply: 6 + GW_MAX_PAYLOAD + 2 = 1032 bytes.
#define STMV2_RX_BUFFER 2048

// ---------------------------------------------------------------------------
// Poll engine
// ---------------------------------------------------------------------------
#define POLL_TASK_STACK 4096
#define POLL_TASK_PRIO 3
#define POLL_TASK_CORE 1  // keep the link off core 0 where the WiFi stack runs

// A tag goes STALE after this long without a successful read, and COMM_FAIL
// after this many consecutive failures.
#define TAG_STALE_MS 15000
#define TAG_FAIL_LIMIT 3

// Downlink write commands queued from the cloud task to the link task.
#define CMD_QUEUE_DEPTH 8
#define ACK_QUEUE_DEPTH 8
// Read the register back after writing it and compare. The STM32 V1.2 always
// answers a write with STATUS_OK even when the field write failed, so a
// read-back is the only way to know the value landed.
#define WRITE_VERIFY 1

// ---------------------------------------------------------------------------
// Cloud
// ---------------------------------------------------------------------------
#ifndef ENABLE_CLOUD
#define ENABLE_CLOUD 1
#endif

#define NET_TASK_STACK 8192
#define NET_TASK_PRIO 2
#define NET_TASK_CORE 0

#define TELEMETRY_INTERVAL_MS 10000  // full tag snapshot publish period
#define STATUS_INTERVAL_MS 60000     // diagnostics publish period
#define MQTT_KEEPALIVE_S 45
#define MQTT_BUFFER_BYTES 3072       // PubSubClient default 256 is far too small
#define WIFI_CONNECT_TIMEOUT_MS 20000
#define RECONNECT_MIN_MS 2000
#define RECONNECT_MAX_MS 60000

// ---------------------------------------------------------------------------
// Housekeeping
// ---------------------------------------------------------------------------
#define STATUS_LED_ENABLE 1  // Freenove board: WS2812 on GPIO48 (RGB_BUILTIN)
#define HEARTBEAT_MS 1000

// Software supervisor: reboot if a task stops checking in. A gateway that
// silently stops reporting is worse than one that reboots.
#define SUPERVISOR_ENABLE 1
#define SUPERVISOR_TIMEOUT_MS 120000
