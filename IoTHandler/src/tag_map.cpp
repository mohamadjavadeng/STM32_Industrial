#include "tag_map.h"

#include <string.h>

#include "stm_protocol.h"

// ---------------------------------------------------------------------------
// EDIT THIS TABLE for your installation.
//
// The addresses below are the ones the STM32 project was bench-tested against
// (see ModbusRTU_AWS/Core/Src/Test.txt): holding registers around 4097 and coils
// around 1281 on slave 2. Treat them as placeholders that are known to respond,
// not as a real map.
//
// Poll cost: each entry is one full request/response over the link, and the
// STM32 answers it by running a live RS-485 transaction. At 9600 baud on the
// field bus that is roughly 25 ms per tag when the slave is healthy, and up to
// 1.6 s when it is not. Size pollMs with the total scan in mind.
// ---------------------------------------------------------------------------
static const TagDef kTags[] = {
    // name              type              addr  scale  offset unit    pollMs  wr     signed
    {"hold_4097",        STM_REG_HOLDING,  4097, 1.0f,  0.0f,  "",      1000,  true,  false},
    {"hold_4098",        STM_REG_HOLDING,  4098, 1.0f,  0.0f,  "",      1000,  false, false},
    // Example of a scaled analog value: raw 0..10000 -> 0.00..100.00 %
    {"level_pct",        STM_REG_HOLDING,  4099, 0.01f, 0.0f,  "%",     2000,  false, false},
    // Example of a signed measurement in tenths of a degree.
    {"temp_c",           STM_REG_INPUT,    4097, 0.1f,  0.0f,  "degC",  2000,  false, true},
    {"pump_run",         STM_REG_COIL,     1281, 1.0f,  0.0f,  "",      1000,  true,  false},
    {"valve_open",       STM_REG_COIL,     1282, 1.0f,  0.0f,  "",      1000,  true,  false},
    {"fault_input",      STM_REG_DISCRETE, 1281, 1.0f,  0.0f,  "",       500,  false, false},
};

const TagDef* tagTable() { return kTags; }

uint16_t tagCount() { return (uint16_t)(sizeof(kTags) / sizeof(kTags[0])); }

int16_t tagIndexOf(const char* name) {
    if (!name) return -1;
    for (uint16_t i = 0; i < tagCount(); ++i) {
        if (strcmp(kTags[i].name, name) == 0) return (int16_t)i;
    }
    return -1;
}
