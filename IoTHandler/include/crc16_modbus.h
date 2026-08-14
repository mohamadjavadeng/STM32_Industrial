/**
 * crc16_modbus.h - CRC-16/MODBUS, identical to the STM32's crc16() in
 * ../ModbusRTU_AWS/Core/Src/modbus_crc.c.
 *
 * That implementation is table driven with confusingly named halves, but its
 * result is the plain standard: poly 0xA001 reflected, init 0xFFFF, no final
 * xor, no reflection of the output. Check value for ASCII "123456789" is 0x4B37.
 *
 * The returned integer's LOW byte is transmitted first, matching both the
 * Modbus RTU specification and what the STM32 does
 * (txBuffer[6] = crc & 0xFF; txBuffer[7] = crc >> 8).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

uint16_t crc16Modbus(const uint8_t* data, size_t len);

/** Appends the CRC little-endian at data[len]. Returns the new total length. */
size_t crc16ModbusAppend(uint8_t* data, size_t len);

/** True if the trailing two little-endian bytes match the CRC of the rest. */
bool crc16ModbusCheck(const uint8_t* frame, size_t totalLen);

/** Self-test against the published check value. Cheap; call it once at boot. */
bool crc16ModbusSelfTest();
