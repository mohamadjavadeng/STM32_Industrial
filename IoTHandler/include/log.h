/**
 * log.h - tiny leveled logger on UART0 (GPIO43/44), the USB-serial console.
 *
 * Deliberately not the core's log_x() macros: those are tied to
 * CORE_DEBUG_LEVEL, which also turns on the WiFi/TLS stack chatter and buries
 * the gateway's own output. Level here is independent and set at build time.
 *
 * Never log to Serial1 - that is the STM32 link.
 */
#pragma once

#include <Arduino.h>

#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_DEBUG 4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#define LOG_PRINT(sev, tag, fmt, ...) \
    Serial.printf("[%8lu] %s %-5s " fmt "\r\n", (unsigned long)millis(), sev, tag, ##__VA_ARGS__)

#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOGE(tag, fmt, ...) LOG_PRINT("E", tag, fmt, ##__VA_ARGS__)
#else
#define LOGE(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOGW(tag, fmt, ...) LOG_PRINT("W", tag, fmt, ##__VA_ARGS__)
#else
#define LOGW(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOGI(tag, fmt, ...) LOG_PRINT("I", tag, fmt, ##__VA_ARGS__)
#else
#define LOGI(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOGD(tag, fmt, ...) LOG_PRINT("D", tag, fmt, ##__VA_ARGS__)
#define LOG_HEXDUMP(tag, label, buf, len)                                       \
    do {                                                                        \
        Serial.printf("[%8lu] D %-5s %s[%u]:", (unsigned long)millis(), tag,     \
                      label, (unsigned)(len));                                  \
        for (size_t _i = 0; _i < (size_t)(len); ++_i)                           \
            Serial.printf(" %02X", ((const uint8_t*)(buf))[_i]);                \
        Serial.println();                                                       \
    } while (0)
#else
#define LOGD(tag, fmt, ...) ((void)0)
#define LOG_HEXDUMP(tag, label, buf, len) ((void)0)
#endif
