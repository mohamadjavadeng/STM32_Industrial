/**
 * gw_link_port.h - the only file in the link module that knows about a UART.
 * ---------------------------------------------------------------------------
 *
 * Everything above this line (gw_link.c) deals in whole frames and never calls
 * HAL. Everything below it is bytes. Keeping the boundary sharp is what makes
 * the transport replaceable: moving the link to a different USART, to SPI, or
 * onto a USB CDC endpoint is a rewrite of this one file with the same four
 * functions, and the protocol engine does not notice.
 *
 * HOW IT DRIVES USART3
 *   RX: one circular DMA stream landing in a private ring buffer, running for
 *       the whole life of the program. Nothing ever stops it, so there is no
 *       window in which bytes can be lost - which was a real defect in v1,
 *       where the STM32 read USART3 one byte at a time from the main loop and
 *       was not reading it at all while a blocking Modbus transaction ran. The
 *       F407 USART has a one-byte hardware buffer, so every ESP32 byte that
 *       arrived in that window was gone.
 *   TX: DMA, one frame at a time. A 1 KB reply is 89 ms at 115200 baud; a
 *       blocking transmit of that length would stall the control loop for
 *       longer than most scan cycles.
 *
 * NO INTERRUPT CODE OF OUR OWN
 *   There is deliberately no IDLE-line interrupt and no callback here, even
 *   though the .ioc has always had the interrupts enabled. Link v2 carries an
 *   explicit length in its header, so the frame boundary is known from the
 *   header alone and IDLE detection buys nothing. The parser polls how far the
 *   DMA has got, which means:
 *     - not one line has to be added to stm32f4xx_it.c, so nothing in this
 *       module can be lost to a CubeMX regeneration
 *     - there is no ISR/mainline shared state to reason about
 *   The cost is a poll, and GwLink_Step() is called every superloop pass
 *   anyway.
 */
#ifndef INC_GW_LINK_PORT_H_
#define INC_GW_LINK_PORT_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h" /* UART_HandleTypeDef */

/**
 * Binds the module to a UART and starts the receive DMA.
 * The handle must already be initialised by CubeMX (MX_USART3_UART_Init) and
 * must have a DMA stream linked for both directions - the .ioc does this
 * already: USART3_RX on DMA1 Stream1 (circular), USART3_TX on DMA1 Stream3.
 *
 * Returns false if the handle has no RX DMA linked, which is the one mistake
 * that would otherwise look like a silent dead link.
 */
bool GwLinkPort_Init(UART_HandleTypeDef* huart);

/**
 * Copies up to max newly arrived bytes out of the DMA ring. Returns how many.
 * Never blocks, never waits, returns 0 when there is nothing new.
 */
uint16_t GwLinkPort_Read(uint8_t* dst, uint16_t max);

/** True while a previous GwLinkPort_Send is still on the wire. */
bool GwLinkPort_TxBusy(void);

/**
 * Starts a DMA transmit. Returns false if a transfer is still running or the
 * HAL refused.
 *
 * The buffer is read by DMA after this call returns, so it must stay valid and
 * unmodified until GwLinkPort_TxBusy() goes false. gw_link.c satisfies that by
 * transmitting only from its own static reply buffer and never building the
 * next reply while the previous one is in flight.
 */
bool GwLinkPort_Send(const uint8_t* data, uint16_t len);

/**
 * Health check: restarts the receive DMA if the HAL aborted it.
 *
 * The F4 HAL treats a UART overrun as a fatal error for the transfer: it ends
 * the RX transfer and disables the DMA request. Without this call the link
 * would go permanently deaf after one noise burst, which is the worst kind of
 * field failure - it looks exactly like a dead cable. Call it every pass; it
 * costs one comparison when all is well.
 */
void GwLinkPort_Service(void);

/** Bytes discarded because the ring overflowed between two Step() calls. */
uint32_t GwLinkPort_OverrunCount(void);

#endif /* INC_GW_LINK_PORT_H_ */
