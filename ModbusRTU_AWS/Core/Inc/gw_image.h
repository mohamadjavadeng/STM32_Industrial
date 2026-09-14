/**
 * gw_image.h - the process image: the only thing that crosses the link.
 * ---------------------------------------------------------------------------
 *
 * WHAT IT IS
 *   A block of RAM holding the current value, timestamp and quality of every
 *   channel the gateway knows about, cut into one region per data source. The
 *   field-bus drivers write it; the link reads it. Neither ever waits for the
 *   other.
 *
 * WHY IT EXISTS
 *   In link v1 an ESP32 read triggered a live Modbus transaction and the ESP32
 *   waited for it - so one unresponsive RS-485 slave cost 1.6 s of link latency
 *   on every scan, the STM32 was not reading USART3 while it waited (so ESP32
 *   bytes were lost), and the STM32 could not do anything else at all. Worse,
 *   the failure was invisible: the reply still said STATUS_OK and carried stale
 *   bytes, so the cloud was told bad data was good.
 *
 *   With an image in between, a read is a bounds-checked memcpy. A dead slave
 *   costs one slot's quality flag. The link's timeout budget stops being
 *   dictated by the slowest device on the field bus.
 *
 * WHO WRITES WHAT (the ownership rule - do not break it)
 *   Exactly one writer per slot, forever:
 *     - the RS-485 scanner owns MB_RTU        (P3)
 *     - the local IO driver owns LOCAL_IO      (P7)
 *     - the CAN driver owns CAN                (P7)
 *     - the ESP32 owns MB_TCP via WR_REGION    (P7, flagged EXTERNAL)
 *     - the virtual evaluator owns VIRTUAL     (P7)
 *     - this module owns SYS
 *   Cloud commands and local control logic do NOT write slots directly. They
 *   queue a write (WR_CHANNEL, P8) and the owning driver executes it. With one
 *   writer there is nothing to arbitrate and no oscillation when two sources
 *   disagree about a setpoint.
 *
 * WHY THERE IS NO LOCK ANYWHERE IN HERE
 *   On Cortex-M4 an aligned 32-bit store is atomic, so a reader can never catch
 *   half a value, and the layout in gw_model.h guarantees the alignment. A
 *   reader *can* catch slot 5 three milliseconds newer than slot 4 - that is
 *   what a process image is, and the timestamp block is how the consumer sees
 *   it. A consumer that needs a consistent set reads the SYS image sequence
 *   counter, reads the block, reads the counter again and retries on a change.
 *   No critical sections, so a slow reader can never delay a scan.
 */
#ifndef INC_GW_IMAGE_H_
#define INC_GW_IMAGE_H_

#include <stdbool.h>
#include <stdint.h>

#include "gw_link_cfg.h"
#include "gw_model.h"

/* ==========================================================================
 * Lifecycle
 * ==========================================================================
 */

/**
 * Lays out the regions, zeroes the arena, sets every quality to
 * GW_Q_UNKNOWN and fills the static part of the SYS region.
 *
 * Call once, from main() before GwLink_Init(). Returns false if the configured
 * regions do not fit in GW_IMAGE_BYTES or if the arena landed somewhere DMA
 * cannot reach; in both cases the image is left disabled rather than truncated,
 * because a half-built image would serve plausible wrong answers.
 */
bool GwImage_Init(void);

/**
 * Housekeeping. Call once per superloop pass - it is cheap and never blocks.
 *   - refreshes uptime and the loop counter in the SYS region
 *   - demotes GOOD slots that have not been refreshed within GW_DEFAULT_STALE_MS
 *   - animates the demo slots when GW_IMAGE_DEMO is 1
 */
void GwImage_Step(void);

/* ==========================================================================
 * Layout enquiry - what RD_META answers from
 * ==========================================================================
 */

uint8_t GwImage_RegionCount(void);

/** Descriptor by region id (GW_REGION_*), or NULL if that region is unknown. */
const GwRegionDesc* GwImage_Desc(uint8_t region);

/** Descriptor by table index 0..GwImage_RegionCount()-1, or NULL. */
const GwRegionDesc* GwImage_DescAt(uint8_t index);

/** Total bytes actually laid out, for the RD_META header. */
uint16_t GwImage_TotalBytes(void);

uint32_t GwImage_ConfigCrc32(void);
uint16_t GwImage_ConfigVersion(void);

/* ==========================================================================
 * Raw byte access - what the link uses
 * ==========================================================================
 * These are the only two functions gw_link.c calls to serve RD_REGION and
 * WR_REGION. Both validate the region and the range before touching memory and
 * return a GW_ST_* code, so the protocol layer never does arithmetic on an
 * address it has not had checked.
 */

/** Copies len bytes from region+off into dst. GW_ST_OK / BAD_REGION / RANGE. */
uint8_t GwImage_Read(uint8_t region, uint16_t off, uint16_t len, uint8_t* dst);

/**
 * Copies len bytes from src into region+off.
 *
 * Refused with GW_ST_NOT_WRITABLE unless the region carries GW_REGF_WRITABLE -
 * which today is MB_TCP alone, because the Modbus TCP master runs on the ESP32
 * and pushes its results in. Everything else is owned by an STM32 driver and
 * must not be writable from the network side: a radio half that can overwrite
 * the control half's process data gives away the isolation the two-chip split
 * was for.
 */
uint8_t GwImage_Write(uint8_t region, uint16_t off, const uint8_t* src, uint16_t len);

/* ==========================================================================
 * Slot access - what drivers use
 * ==========================================================================
 * A driver never computes an offset. It says "slot 12 of MB_RTU is now 4321 and
 * good", and the layout stays an implementation detail of this file.
 *
 * Every setter stamps HAL_GetTick() into the slot's timestamp, which is what
 * keeps the staleness logic honest: forgetting to stamp is impossible.
 */

/** Raw 32-bit store. Use for u16/i16/u32/i32/bool after widening. */
uint8_t GwImage_SetU32(uint8_t region, uint16_t slot, uint32_t value, uint8_t quality);

/** Float store. Bit pattern, not a conversion - the ESP32 reads it back as f32. */
uint8_t GwImage_SetF32(uint8_t region, uint16_t slot, float value, uint8_t quality);

/**
 * Marks a slot bad without touching its value.
 *
 * This is the call a driver makes when a device stops answering, and it is the
 * whole point of the quality byte: the last known value stays visible (useful
 * for a human), while everything downstream can see it is not to be trusted.
 * The cloud layer must publish quality alongside the number, or publish
 * nothing - see the quality section of gw_model.h.
 */
uint8_t GwImage_SetQuality(uint8_t region, uint16_t slot, uint8_t quality);

/** Reads one slot back. Any of the three out pointers may be NULL. */
uint8_t GwImage_GetSlot(uint8_t region, uint16_t slot, uint32_t* value, uint32_t* stampMs,
                        uint8_t* quality);

/* ==========================================================================
 * SYS region helpers
 * ==========================================================================
 * The SYS region is a plain struct of diagnostics at fixed offsets
 * (GW_SYS_OFF_* in gw_model.h). The link module keeps its own counters there so
 * that "is this gateway healthy" is answerable from the cloud without adding a
 * single protocol message.
 */

void GwImage_SysSetU32(uint16_t off, uint32_t value);
void GwImage_SysSetU16(uint16_t off, uint16_t value);
void GwImage_SysSetU8(uint16_t off, uint8_t value);
uint32_t GwImage_SysGetU32(uint16_t off);

/** Sets / clears bits in GW_SYS_OFF_FAULT_BITS. Latching is the caller's call. */
void GwImage_SetFault(uint32_t bits);
void GwImage_ClearFault(uint32_t bits);

#endif /* INC_GW_IMAGE_H_ */
