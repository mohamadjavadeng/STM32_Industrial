/**
 * gw_link.h - the protocol engine of the STM32 half of the link.
 * ---------------------------------------------------------------------------
 *
 * Parses link v2 requests, answers them from the process image, and never
 * blocks. Two calls make up the entire public surface:
 *
 *   GwLink_Init(&huart3);   once, after the peripherals are up
 *   GwLink_Step();          every superloop pass
 *
 * The STM32 is a pure responder here. It never speaks first, which is not an
 * accident of the implementation but the load-bearing decision of the whole
 * architecture: the ESP32 is the side that is unavailable at unpredictable
 * times (WiFi roaming, TLS handshakes, broker backoff, OTA), so the side that
 * owns the process data answers rather than pushes. A responder's worst case is
 * one reply. A pusher's worst case is an unbounded retry queue on the chip that
 * is running the control loop.
 *
 * The one exception, when it is built (P8), is the STM_EVENT GPIO: a single
 * wire the STM32 raises to say "poll me now, do not wait for your next tick".
 * That keeps a strictly single-initiator protocol while still giving
 * millisecond alarm latency. The ESP32 already supports it through
 * PIN_STM_EVENT; no STM32 firmware has ever driven it.
 *
 * WHAT STEP() COSTS
 *   With nothing arriving: one DMA counter read and a comparison. With a
 *   request pending: parsing is a memcpy and a CRC over the frame, the handler
 *   is a bounds check and a memcpy out of the image, and the reply leaves by
 *   DMA. There is no path through this file that waits on anything - no field
 *   bus, no timeout, no HAL_Delay. That property is the reason the link's
 *   response timeout on the ESP32 side can drop from 1600 ms to about 50.
 */
#ifndef INC_GW_LINK_H_
#define INC_GW_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "gw_link_cfg.h"
#include "main.h"

/**
 * Counters for the health of the link itself, mirrored into the SYS region so
 * the ESP32 - and the cloud behind it - can see them without a special
 * message. Read them live in a debugger through the .launch file's live
 * expressions, or over the wire with RD_REGION on region 0x00.
 */
typedef struct {
    uint32_t rxFrames;       /* requests accepted and dispatched              */
    uint32_t txFrames;       /* replies handed to the DMA                     */
    uint32_t crcErrors;      /* frame reached us complete but corrupted       */
    uint32_t frameErrors;    /* impossible length, or a frame that never ended */
    uint32_t droppedBytes;   /* bytes thrown away while hunting for 0xA5      */
    uint32_t dupRetransmits; /* replies re-sent from the cache for a repeat SEQ */
    uint32_t lastRxMs;       /* HAL_GetTick() of the last accepted request    */
} GwLinkStats;

/**
 * Binds the link to a UART and starts receiving.
 *
 * GwImage_Init() must have succeeded first: the link answers out of the image
 * and reports its own counters into it. Returns false if the image is not up or
 * the port could not start, and in that case GwLink_Step() does nothing at all
 * rather than answering with a half-initialised image.
 */
bool GwLink_Init(UART_HandleTypeDef* huart);

/**
 * Consumes whatever the DMA has received, dispatches any complete request, and
 * sends the reply. Call it every pass of the superloop; call it more often than
 * anything else if the loop ever grows long, because it is the only thing in
 * the firmware with a peer waiting on it.
 */
void GwLink_Step(void);

/** Live counters. Never NULL. */
const GwLinkStats* GwLink_Stats(void);

/** True once Init has succeeded. */
bool GwLink_IsReady(void);

#endif /* INC_GW_LINK_H_ */
