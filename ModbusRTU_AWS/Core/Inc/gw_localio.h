/**
 * gw_localio.h - the driver that owns this module's own terminals.
 * ---------------------------------------------------------------------------
 *
 * WHAT IT DRIVES
 *   PD0..PD3  four relay outputs
 *   PD4..PD7  four digital inputs, internally pulled up
 *
 * WHERE THE VALUES GO
 *   Into region GW_REGION_LOCAL_IO of the process image, at the fixed slots
 *   defined in shared/gw_model.h (section 10). Nothing else in the firmware
 *   writes those slots - that is the ownership rule from gw_image.h, and it is
 *   what lets this driver run with no lock against a link that reads the same
 *   memory from an interrupt.
 *
 * HOW A RELAY GETS SWITCHED FROM THE CLOUD
 *   The ESP32 sends WR_CHANNEL naming region LOCAL_IO and a slot. gw_link.c
 *   hands it to GwLocalIO_WriteSlot(), which validates it and stores it as the
 *   new commanded state. The pin changes on the next GwLocalIO_Step().
 *
 *   The cloud never writes the image. It asks this driver to, and this driver
 *   is the only thing that ever stores to those slots, so the value the cloud
 *   reads back is always what the pin is actually doing rather than what
 *   something once asked it to do. When a write is refused - bad slot, bad
 *   encoding, an input slot - the refusal comes back as a GW_ST_* code, not as
 *   a silent no-op.
 *
 * POLARITY LIVES IN ONE PLACE
 *   Cheap opto-isolated relay boards energize on a LOW input; MOSFET and SSR
 *   boards usually energize on HIGH. Dry-contact inputs to a pulled-up pin read
 *   LOW when the contact is closed. All three inversions are single #defines in
 *   gw_link_cfg.h, and every layer above this file - image, link, cloud,
 *   dashboard - deals only in logical ON/OFF. Getting polarity wrong should
 *   cost one constant, not a hunt through four components.
 */
#ifndef INC_GW_LOCALIO_H_
#define INC_GW_LOCALIO_H_

#include <stdbool.h>
#include <stdint.h>

#include "gw_link_cfg.h"
#include "gw_model.h"

/**
 * Drives every relay to its de-energized state and seeds the image slots.
 *
 * Call from main() after MX_GPIO_Init() and GwImage_Init(), and before
 * GwLink_Init() - so the pins exist before this touches them, and no LOCAL_IO
 * slot is ever served to the ESP32 before it has been written once.
 *
 * WHO CONFIGURES THE PINS
 *   MX_GPIO_Init(), generated from ModbusRTU_AWS.ioc, which carries PD0..PD3 as
 *   GPIO_Output (PinState GPIO_PIN_SET - de-energized for an active-low board)
 *   and PD4..PD7 as GPIO_Input with GPIO_PULLUP. The .ioc is this project's
 *   source of truth for peripheral init; a driver configuring its own pins
 *   would be a second answer to the same question, invisible in CubeMX, and
 *   silently wrong the day someone reassigns one of those pins there.
 *
 * Returns false if the image has no LOCAL_IO region or it is too small for the
 * slot map, in which case the driver disables itself instead of writing past
 * the region.
 */
bool GwLocalIO_Init(void);

/**
 * Samples the inputs, applies any pending relay command, and refreshes the
 * image. Call once per superloop pass; it never blocks.
 *
 * Inputs are debounced by GW_LIO_DEBOUNCE_MS: a reading has to agree with
 * itself for that long before it reaches the image. Without it a relay
 * contact's bounce would publish a burst of transitions to the cloud, and an
 * operator would see a switch that flickers when nothing moved.
 */
void GwLocalIO_Step(void);

/**
 * Applies one WR_CHANNEL to a LOCAL_IO slot. Called only by gw_link.c.
 *
 * Accepts slot GW_LIO_DO_BASE..+3 (one relay, value 0 or 1) and
 * GW_LIO_DO_WORD (all four at once, bit 0 = relay 1). Refuses the input slots
 * with GW_ST_NOT_WRITABLE - they are what this driver reports, not what it
 * accepts - and refuses a float with GW_ST_BAD_LEN, because a relay has no
 * meaningful value between 0 and 1 and silently rounding one would be a guess
 * at what the operator meant.
 *
 * Returns a GW_ST_* code. The pin moves on the next GwLocalIO_Step(), which is
 * at most one superloop pass away.
 */
uint8_t GwLocalIO_WriteSlot(uint16_t slot, uint32_t value, uint8_t encoding);

/** Commanded relay bitmask, bit 0 = relay 1. Reflects the last accepted write. */
uint8_t GwLocalIO_Relays(void);

/** Debounced input bitmask, bit 0 = input 1. */
uint8_t GwLocalIO_Inputs(void);

/**
 * Drops every relay and marks the commanded state as failsafe.
 *
 * Exposed so that something above this driver - a lost-link watchdog, a local
 * estop, a fault handler - can put the outputs in a known state without
 * knowing how they are wired. The automatic version of this is
 * GW_LIO_FAILSAFE_MS.
 */
void GwLocalIO_Failsafe(void);

#endif /* INC_GW_LOCALIO_H_ */
