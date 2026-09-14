/**
 * gw_link_port.c - UART/DMA transport for the link module.
 *
 * See gw_link_port.h for the reasoning. This file contains no protocol
 * knowledge at all: it moves bytes and keeps the DMA alive.
 */
#include "gw_link_port.h"

#include <string.h>

#include "gw_image.h"
#include "gw_link_cfg.h"

/* Ring the RX DMA writes into, continuously and forever.
 *
 * The size must be a power of two: the read path masks instead of dividing,
 * and a mask is one instruction where a modulo on a Cortex-M4 is a division.
 * The static assert below is a compile-time trip wire for anyone who edits
 * GW_LINK_RX_DMA_BYTES to a round decimal number. */
static uint8_t gRxRing[GW_LINK_RX_DMA_BYTES];
#define RX_RING_MASK (GW_LINK_RX_DMA_BYTES - 1u)

typedef char gw_assert_ring_pow2[((GW_LINK_RX_DMA_BYTES & RX_RING_MASK) == 0u) ? 1 : -1];

static UART_HandleTypeDef* gUart;
static uint16_t gTail;          /* next byte we have not consumed yet          */
static uint32_t gOverrunCount;  /* times the ring lapped us                    */
static bool gReady;

/* --------------------------------------------------------------------------
 * Where the DMA has got to.
 * --------------------------------------------------------------------------
 * A circular DMA stream counts DOWN from the buffer size, so the write head is
 * size - NDTR. Reading NDTR is a single 32-bit load of a register the DMA
 * updates atomically, so no interrupt has to be masked to read it.
 */
static uint16_t dmaHead(void) {
    uint32_t remaining = __HAL_DMA_GET_COUNTER(gUart->hdmarx);
    if (remaining > GW_LINK_RX_DMA_BYTES) remaining = GW_LINK_RX_DMA_BYTES; /* paranoia */
    return (uint16_t)((GW_LINK_RX_DMA_BYTES - remaining) & RX_RING_MASK);
}

/** (Re)starts the circular receive. Resets the read cursor to match. */
static bool startRx(void) {
    gTail = 0u;
    /* Aborting first matters on the re-arm path: the HAL may believe a transfer
     * is still live, and HAL_UART_Receive_DMA would then return BUSY and leave
     * the link deaf forever. */
    (void)HAL_UART_AbortReceive(gUart);
    return HAL_UART_Receive_DMA(gUart, gRxRing, GW_LINK_RX_DMA_BYTES) == HAL_OK;
}

bool GwLinkPort_Init(UART_HandleTypeDef* huart) {
    gReady = false;
    gOverrunCount = 0u;

    if (huart == 0 || huart->hdmarx == 0 || huart->hdmatx == 0) {
        /* Missing DMA link. This happens when CubeMX is regenerated with the
         * DMA settings removed from the .ioc, and it would otherwise present as
         * a link that never receives anything - hours of cable swapping for a
         * checkbox. Fail loudly instead. */
        return false;
    }

    gUart = huart;
    if (!startRx()) return false;

    gReady = true;
    return true;
}

uint16_t GwLinkPort_Read(uint8_t* dst, uint16_t max) {
    uint16_t head;
    uint16_t available;
    uint16_t n;

    if (!gReady || max == 0u) return 0u;

    head = dmaHead();
    available = (uint16_t)((head - gTail) & RX_RING_MASK);
    if (available == 0u) return 0u;

    /*
     * Overrun detection is necessarily approximate on a circular DMA: the
     * hardware does not tell us it lapped, it just overwrites. If the unread
     * backlog has grown past three quarters of the ring, the caller is being
     * starved of CPU badly enough that data loss is imminent, so say so.
     *
     * At 115200 baud this needs GwLink_Step() to go unserviced for 133 ms.
     * If it ever fires in the field, something in the superloop is blocking -
     * which is exactly the class of bug the fault bit is there to expose.
     */
    if (available > (GW_LINK_RX_DMA_BYTES - (GW_LINK_RX_DMA_BYTES / 4u))) {
        gOverrunCount++;
        GwImage_SetFault(GW_FAULT_LINK_OVERRUN);
    }

    n = (available < max) ? available : max;

    /* One or two memcpys depending on whether the run wraps the end. */
    if ((uint32_t)gTail + n <= GW_LINK_RX_DMA_BYTES) {
        memcpy(dst, &gRxRing[gTail], n);
    } else {
        uint16_t first = (uint16_t)(GW_LINK_RX_DMA_BYTES - gTail);
        memcpy(dst, &gRxRing[gTail], first);
        memcpy(dst + first, &gRxRing[0], (uint16_t)(n - first));
    }

    gTail = (uint16_t)((gTail + n) & RX_RING_MASK);
    return n;
}

bool GwLinkPort_TxBusy(void) {
    if (!gReady) return false;
    return gUart->gState != HAL_UART_STATE_READY;
}

bool GwLinkPort_Send(const uint8_t* data, uint16_t len) {
    if (!gReady || len == 0u) return false;
    if (GwLinkPort_TxBusy()) return false;

    /* The cast drops const because the HAL prototype predates const-correctness;
     * the driver only reads the buffer. */
    return HAL_UART_Transmit_DMA(gUart, (uint8_t*)(uintptr_t)data, len) == HAL_OK;
}

void GwLinkPort_Service(void) {
    if (!gReady) return;

    /*
     * The F4 HAL classifies a UART overrun as a blocking error: HAL_UART_IRQHandler
     * calls UART_EndRxTransfer() and disables the DMA request, leaving RxState
     * back at READY. Nothing restarts it on its own, so without this check one
     * noise burst on the cable deafens the link until the next power cycle.
     */
    if (gUart->RxState != HAL_UART_STATE_BUSY_RX) {
        GwImage_SetFault(GW_FAULT_LINK_OVERRUN);
        gOverrunCount++;
        (void)startRx();
        /* The in-flight frame, if any, is lost. The ESP32 will time out and
         * retry; that is the designed recovery and it costs one transaction. */
    }
}

uint32_t GwLinkPort_OverrunCount(void) { return gOverrunCount; }
