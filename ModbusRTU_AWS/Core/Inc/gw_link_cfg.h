/**
 * gw_link_cfg.h - every tunable of the STM32 half of the link module.
 * ---------------------------------------------------------------------------
 *
 * One file to look in when something needs resizing or retiming. Nothing here
 * changes the wire protocol - that is shared/gw_model.h and changing it means
 * reflashing both chips. Everything here is local: buffer sizes, how many slots
 * each region gets, and how the UART is driven.
 *
 * The module is four files plus this one:
 *
 *   gw_model.h      the wire contract, shared with the ESP32
 *   gw_link_cfg.h   you are here - local tunables
 *   gw_image.[ch]   the process image: regions, slots, quality, timestamps
 *   gw_link.[ch]    the protocol engine: parse, dispatch, build replies
 *   gw_link_port.[ch] the UART binding: DMA in, DMA out, nothing else
 *
 * The split exists so each piece can be replaced alone. Moving from USART3 to
 * another UART, or to SPI, touches gw_link_port.c only. Adding an opcode
 * touches gw_link.c only. Adding a driver (CAN, local IO) touches neither: a
 * driver writes slots through gw_image.h and the link serves them unchanged.
 */
#ifndef INC_GW_LINK_CFG_H_
#define INC_GW_LINK_CFG_H_

#include "gw_model.h"

/* ==========================================================================
 * Master switch
 * ==========================================================================
 * 1 = USART3 belongs to link v2 (this module).
 * 0 = USART3 belongs to the old 0xAA/0xBB handler in esp32msghandler.c.
 *
 * They cannot share the port. v1 services USART3 with a blocking
 * HAL_UART_Receive of one byte at a time; v2 owns the RX DMA stream for the
 * whole run. Pick one at build time - main.c honours this flag.
 *
 * The migration path in docs/UNIVERSAL_PROCESS_IMAGE.html suggests running both
 * parsers side by side off one byte stream. That is doable later (dispatch on
 * the first byte: 0xAA to v1, 0xA5 to v2) and worth it when there is a fleet in
 * the field. On the bench it only doubles what can go wrong during bring-up, so
 * the first step is a clean switch.
 */
#ifndef GW_LINK_ENABLE
#define GW_LINK_ENABLE 1
#endif

/* ==========================================================================
 * Firmware identity - reported by SYNC and in the SYS region
 * ==========================================================================
 * Bump this when you flash something new, so a field unit can be identified
 * from the cloud without a probe. It tracks the repository tag (V1.3 was the
 * last release of the v1 link).
 */
#define GW_FW_MAJOR 1
#define GW_FW_MINOR 4
#define GW_FW_PATCH 0

/*
 * Identity of the configuration this firmware is running.
 *
 * Today the channel layout is compiled in, so this is a constant that changes
 * only when a developer edits the region table below. From phase P5 it becomes
 * the CRC32 of the config blob in flash, and the PC tool stamps the identical
 * value into the ESP32's tag map. The ESP32 compares the two and refuses to
 * publish on a mismatch - see the GwMetaHeader comment in gw_model.h for why
 * that interlock is not optional.
 *
 * Rule for now: change the region table below -> change this constant.
 */
#define GW_CONFIG_CRC32 0xC0DE0001u
#define GW_CONFIG_VERSION 1u

/* ==========================================================================
 * Process image sizing
 * ==========================================================================
 * A structured region costs slots * 9 bytes (4 value + 4 timestamp + 1
 * quality). A raw region costs exactly what it asks for.
 *
 * Defaults below total 6464 bytes. On a 128 KB part that is not worth
 * optimising, and headroom now avoids a protocol-visible resize later - though
 * a resize is survivable, because the ESP32 reads the layout from RD_META
 * instead of hardcoding it.
 *
 * The image must live in normal SRAM (0x20000000), never in the 64 KB CCM at
 * 0x10000000: CCM is not reachable by DMA on the STM32F4, and serving the link
 * straight out of the image by DMA is the whole point. Default .bss is in RAM,
 * so this happens by itself - GwImage_Init() asserts it anyway, because the day
 * someone adds a linker section for CCM the failure would otherwise be a DMA
 * that silently transfers nothing.
 */
#define GW_SLOTS_LOCAL_IO 64u  /* DI / DO / AI / AO of this module (P7)      */
#define GW_SLOTS_MB_RTU 256u   /* RS-485 field devices - populated first (P3) */
#define GW_SLOTS_MB_TCP 128u   /* filled by the ESP32 over WR_REGION (P7)    */
#define GW_SLOTS_CAN 128u      /* decoded CAN signals (P7)                    */
#define GW_SLOTS_SERIAL 64u    /* custom serial devices (P7)                  */
#define GW_SLOTS_VIRTUAL 64u   /* computed tags (P7)                          */
#define GW_BYTES_EVENTS 0u     /* event ring, sized in P8; 0 = region disabled */

/* Total arena. Must be >= the sum of the regions above, each padded up to a
 * 4-byte boundary. GwImage_Init() refuses to run if it does not fit, rather
 * than quietly truncating the last region. */
#define GW_IMAGE_BYTES 8192u

/* ==========================================================================
 * Staleness
 * ==========================================================================
 * A slot whose timestamp is older than this is demoted GOOD -> STALE by
 * GwImage_Step(). Per-channel staleMs arrives with the config blob in P5; until
 * then one number covers everything.
 *
 * This is the STM32-side half of the quality story. The ESP32 has its own
 * ageing in process_image.cpp, which exists only because link v1 could not
 * report quality at all. Once the ESP32 consumes the quality byte from the
 * image, that duplicate logic should go: two components ageing the same value
 * with two different clocks is a bug waiting for a slow scan to expose it.
 */
#define GW_DEFAULT_STALE_MS 15000u

/* ==========================================================================
 * Link timing and buffers
 * ==========================================================================
 */

/* Circular DMA landing buffer for USART3 RX. It only has to cover the gap
 * between two GwLink_Step() calls, so it is generous by a wide margin: 2048
 * bytes is 178 ms of traffic at 115200 baud. Must be a power of two - the ring
 * arithmetic in gw_link_port.c depends on it. */
#define GW_LINK_RX_DMA_BYTES 2048u

/* Working buffers for one request and one reply. One of each: the protocol is
 * strictly one transaction in flight. */
#define GW_LINK_RX_FRAME_BYTES (GW_REQ_HEADER_LEN + GW_MAX_PAYLOAD + GW_CRC_LEN)
#define GW_LINK_TX_FRAME_BYTES (GW_RSP_HEADER_LEN + GW_MAX_PAYLOAD + GW_CRC_LEN)

/*
 * If a frame has started and no further byte arrives within this long, the
 * parser throws away what it has and starts hunting for SOF again.
 *
 * This is the fix for a specific v1 failure: a truncated request stayed in the
 * buffer forever and was re-parsed - and re-executed - when the next unrelated
 * byte arrived. With an explicit LEN and this timeout, a truncated frame costs
 * one timeout and nothing else.
 *
 * 50 ms is far longer than any legitimate inter-byte gap (87 us at 115200) and
 * far shorter than the ESP32's response timeout, so the STM32 has always
 * recovered before the ESP32 retries.
 */
#define GW_LINK_FRAME_IDLE_MS 50u

/*
 * Remember the last answered SEQ and its reply, and re-send that reply if the
 * same SEQ arrives again instead of executing the request a second time.
 *
 * This is what makes a retry safe. The ESP32 retries when a reply is lost, and
 * the request may well have been executed already - a WR_REGION replayed
 * blindly would apply twice. Answering from the cache keeps a retry idempotent
 * for every opcode, at the cost of the buffer that is allocated anyway.
 *
 * SEQ 0 opts out, for a caller that genuinely wants the request re-executed.
 */
#define GW_LINK_DEDUP_ENABLE 1

/* ==========================================================================
 * Bench aids
 * ==========================================================================
 */

/*
 * 1 = GwImage_Step() animates a handful of slots so the link can be tested with
 * no RS-485 slave attached: a counter, a triangle wave, a float, a toggling
 * bool, and one slot deliberately left COMM_FAIL so the quality path is visible
 * end to end.
 *
 * Set to 0 as soon as the real scanner (P3) writes these slots. Leaving it on
 * would have two writers on one slot, which is exactly the ownership rule the
 * design forbids.
 */
#ifndef GW_IMAGE_DEMO
#define GW_IMAGE_DEMO 1
#endif

#endif /* INC_GW_LINK_CFG_H_ */
