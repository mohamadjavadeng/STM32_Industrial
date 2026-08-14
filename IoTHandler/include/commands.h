/**
 * commands.h - downlink write commands travelling between the two tasks.
 *
 * The MQTT callback runs on the network task and must return quickly, so it only
 * parses and enqueues. The link task owns the serial port and executes. Results
 * come back on a second queue for the network task to publish.
 *
 * Fixed-size POD, copied by value into FreeRTOS queues - no pointers, no
 * allocation, nothing to free on either side.
 */
#pragma once

#include <stdint.h>

#define CMD_ID_LEN 24
#define CMD_TAG_LEN 24
#define CMD_DETAIL_LEN 40

enum CmdKind : uint8_t {
    CMD_WRITE_TAG = 0,  // write by tag name from the table
    CMD_WRITE_RAW = 1,  // write by explicit register class + address
    CMD_PING = 2,       // liveness probe of the STM32 link
};

struct LinkCommand {
    char id[CMD_ID_LEN];    // echoed in the ack so the caller can correlate
    char tag[CMD_TAG_LEN];  // CMD_WRITE_TAG only
    uint8_t kind;
    uint8_t regType;   // CMD_WRITE_RAW only
    uint16_t address;  // CMD_WRITE_RAW only
    float value;       // engineering units unless valueIsRaw
    bool valueIsRaw;   // true = write straight to the register, no scale/offset
};

struct LinkAck {
    char id[CMD_ID_LEN];
    char tag[CMD_TAG_LEN];
    char detail[CMD_DETAIL_LEN];
    uint8_t kind;
    uint8_t status;   // StmStatus
    bool accepted;    // false = rejected before it reached the wire
    uint16_t written; // raw value actually sent
};
