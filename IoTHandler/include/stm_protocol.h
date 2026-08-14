/**
 * stm_protocol.h - wire format of the ESP32 <-> STM32 link.
 *
 * This is a mirror of ../ModbusRTU_AWS/Core/Inc/esp32msghandler.h. Keep the two
 * in sync; the constants below are the contract.
 *
 * FRAMING
 *   request   AA cmd type addrHi addrLo count [data...] crcLo crcHi
 *   response  BB status len   [data...]                 crcLo crcHi
 *
 *   CRC is CRC-16/MODBUS (poly 0xA001 reflected, init 0xFFFF) over every byte
 *   before it, transmitted low byte first. Verified byte-for-byte against the
 *   STM32's table implementation in Core/Src/modbus_crc.c.
 *
 *   There is no length field in the request header, so the STM32 infers the
 *   frame length: 6 + (cmd == WRITE ? count : 0) + 2. Only WRITE carries a
 *   payload. Responses do have an explicit payload length at byte 2, which is
 *   what lets one parser handle every reply shape.
 *
 * TRANSACTION MODEL
 *   Strictly one request in flight, ESP32 always initiates. The STM32 has no
 *   inter-frame timeout and services the link by polling one byte at a time
 *   from its main loop, so it cannot absorb pipelined or overlapping requests -
 *   never send a second frame before the first is answered or has timed out.
 *
 * KNOWN QUIRKS OF STM32 FIRMWARE V1.2 (guarded by STM_FW_QUIRKS)
 *
 *   Q1  Swapped register type on single reads. In ESP32MsgHandler_ReadRegister
 *       (esp32msghandler.c:44) REG_TYPE_HOLDING dispatches to
 *       Modbus_ReadInputRegisters (FC 04) and REG_TYPE_INPUT dispatches to
 *       Modbus_ReadHoldingRegisters (FC 03). To read a holding register with
 *       cmd READ you must therefore send type 0x02. ESP32MsgHandler_
 *       MultipleReadRegister does NOT have the swap, so the meaning of the type
 *       byte differs between cmd 0x01 and cmd 0x03.
 *
 *   Q2  count is bytes on the wire but registers on the field bus. For MULREAD
 *       the STM32 puts `count` payload bytes in the response yet asks the
 *       RS-485 slave for `count` *registers*. Requesting N registers means
 *       sending count = 2N, and the STM32 will over-read 2N registers from the
 *       field device (which fails if the slave does not have that many). The
 *       reply carries only the first N. Cap: count <= 100, because the STM32
 *       decodes into uint16_t holdingRegisters[100].
 *
 *   Q3  MULREAD is only usable with type HOLDING. The INPUT branch truncates
 *       uint16 registers into uint8 slots and walks the source with a buffer
 *       index; the COIL/DISCRETE branches copy the packed Modbus bitmask bytes
 *       as if they were one-byte-per-bit and are off by three. All three
 *       produce garbage.
 *
 *   Q4  A failed MULREAD desyncs the parser. esp32msghandler.c:186 does
 *       `continue` without resetting rxIndex, so the stale frame stays in the
 *       STM32 buffer: the next byte received re-parses and re-executes the
 *       previous request, then clears. StmLink::resync() deliberately feeds one
 *       dummy byte to flush that ghost. Safe because only the read-only MULREAD
 *       path can get stuck there.
 *
 *   Q5  Status is not a result code. Both READ and WRITE answer STATUS_OK even
 *       when the underlying Modbus transaction returned a timeout or CRC error -
 *       statusModbus is stored in a global and never sent. STATUS_ERROR appears
 *       only when the STM32 rejects *our* frame's CRC. So an OK reply means
 *       "frame understood", not "field data is valid". Read-back verification
 *       (WRITE_VERIFY) and staleness tracking exist because of this.
 *
 *   Q6  MULWRITE (0x04) is defined but unimplemented - it falls through with no
 *       reply. Never send it.
 *
 *   Q7  A WRITE with count < 2 is silently dropped, no reply. Always send
 *       count = 2 with a big-endian 16-bit value.
 *
 *   Q8  The STM32 receive buffer is 64 bytes and rxIndex is never bounds
 *       checked, so a frame claiming count > 56 overflows it. Requests are
 *       capped at STM_MAX_REQUEST_LEN here; line noise can still trigger it,
 *       which is a reason to fix the STM32 side rather than rely on the client.
 */
#pragma once

#include <stdint.h>

#include "config.h"  // STM_FW_QUIRKS

// ---- Framing --------------------------------------------------------------
#define STM_START_REQ 0xAA
#define STM_START_RSP 0xBB

#define STM_REQ_HEADER_LEN 6  // start, cmd, type, addrHi, addrLo, count
#define STM_RSP_HEADER_LEN 3  // start, status, len
#define STM_CRC_LEN 2

// STM32 rxBuffer is ESP32MSG_BUFFER_SIZE (64) with no bounds check (Q8).
#define STM_MAX_REQUEST_LEN 56
// Largest sane MULREAD: count <= 100 -> 50 registers (Q2).
#define STM_MAX_BLOCK_REGS 50
#define STM_MAX_RSP_PAYLOAD 255

// ---- Commands -------------------------------------------------------------
#define STM_CMD_READ 0x01
#define STM_CMD_WRITE 0x02
#define STM_CMD_MULREAD 0x03
#define STM_CMD_MULWRITE 0x04  // unimplemented on the STM32 (Q6) - do not send

// ---- Register classes -----------------------------------------------------
// These are the logical values used throughout this firmware. The byte that
// actually goes on the wire comes from stmWireTypeForSingleRead() below.
#define STM_REG_HOLDING 0x01
#define STM_REG_INPUT 0x02
#define STM_REG_COIL 0x03
#define STM_REG_DISCRETE 0x04

// ---- Status ---------------------------------------------------------------
#define STM_STATUS_OK 0x00
#define STM_STATUS_ERROR 0x01

/** Q1: translate a logical register class into the type byte cmd READ expects. */
static inline uint8_t stmWireTypeForSingleRead(uint8_t logicalType) {
#if STM_FW_QUIRKS
    if (logicalType == STM_REG_HOLDING) return STM_REG_INPUT;
    if (logicalType == STM_REG_INPUT) return STM_REG_HOLDING;
#endif
    return logicalType;
}

/** Writes are dispatched correctly on the STM32, so no translation. */
static inline uint8_t stmWireTypeForWrite(uint8_t logicalType) {
    return logicalType;
}

/** Q3: only HOLDING survives the MULREAD path. */
static inline bool stmBlockReadSupported(uint8_t logicalType) {
#if STM_FW_QUIRKS
    return logicalType == STM_REG_HOLDING;
#else
    return logicalType == STM_REG_HOLDING || logicalType == STM_REG_INPUT;
#endif
}

static inline const char* stmRegTypeName(uint8_t logicalType) {
    switch (logicalType) {
        case STM_REG_HOLDING: return "holding";
        case STM_REG_INPUT: return "input";
        case STM_REG_COIL: return "coil";
        case STM_REG_DISCRETE: return "discrete";
        default: return "?";
    }
}
