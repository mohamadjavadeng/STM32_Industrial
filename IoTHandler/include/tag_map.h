/**
 * tag_map.h - declarative list of what this gateway reads and writes.
 *
 * One entry per register or bit. Editing this table is the only thing needed to
 * change what ends up in the cloud; nothing else in the firmware knows the
 * addresses. Keep it in step with the field device's register map.
 *
 * Addresses go straight through to the RS-485 slave that the STM32 polls, at
 * whatever offset convention that device uses (the STM32 does not subtract 1).
 * Slave ID is fixed at 2 in the STM32 firmware (`SlaveID` in main.c).
 */
#pragma once

#include <stdint.h>

struct TagDef {
    const char* name;   // cloud-facing identifier; must be unique
    uint8_t regType;    // STM_REG_HOLDING / _INPUT / _COIL / _DISCRETE
    uint16_t address;   // register or bit address on the field device
    float scale;        // engineering value = raw * scale + offset
    float offset;
    const char* unit;   // free text, published as-is ("" for none)
    uint32_t pollMs;    // how often to refresh
    bool writable;      // may a cloud command write it
    bool isSigned;      // interpret raw as int16 rather than uint16
};

const TagDef* tagTable();
uint16_t tagCount();

/** Index of a tag by name, or -1. */
int16_t tagIndexOf(const char* name);
