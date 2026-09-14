/**
 * gw_link.c - link v2 protocol engine.
 *
 * Structure of this file, top to bottom:
 *   1. state          parser state machine, buffers, dedup cache
 *   2. reply builder  one function, so every reply is framed identically
 *   3. handlers       one per opcode, each returning a GW_ST_* code
 *   4. dispatch       opcode -> handler
 *   5. parser         byte in, complete frame out
 *   6. public API     Init / Step / Stats
 *
 * Nothing here calls HAL except through gw_link_port.h, and nothing here knows
 * what a Modbus register is. Both are deliberate: see the header.
 */
#include "gw_link.h"

#include <string.h>

#include "gw_image.h"
#include "gw_link_port.h"
#include "gw_model.h"
#include "modbus_crc.h" /* crc16() - the same CRC the RS-485 side uses */

#if GW_LINK_ENABLE

/* ==========================================================================
 * 1. State
 * ==========================================================================
 */

/**
 * Parser states. The whole point of the explicit LEN field in the v2 header is
 * that these three states are all that is needed, for every opcode, with no
 * lookahead and no per-command special case.
 */
typedef enum {
    LP_HUNT = 0, /* discarding bytes until a 0xA5 turns up          */
    LP_HEADER,   /* collecting the 8 header bytes                    */
    LP_BODY      /* collecting LEN payload bytes plus the 2 CRC bytes */
} LinkParseState;

static LinkParseState gState;
static uint8_t gFrame[GW_LINK_RX_FRAME_BYTES]; /* request under construction  */
static uint16_t gFrameLen;                     /* bytes of gFrame filled      */
static uint16_t gExpect;                       /* total bytes this frame needs */
static uint32_t gFrameStartMs;                 /* for the idle timeout        */

static uint8_t gTx[GW_LINK_TX_FRAME_BYTES] __attribute__((aligned(4)));
static uint16_t gTxLen;

static GwLinkStats gStats;
static bool gReady;

/* Bytes pulled out of the DMA ring but not yet fed to the parser.
 *
 * Needed because feeding can stop mid-chunk: as soon as a frame completes and a
 * reply goes out, the rest of the chunk has to wait until the transmitter is
 * free again, because dispatch() would overwrite the buffer the DMA is reading.
 * Without somewhere to park them those bytes would be dropped. The protocol
 * forbids pipelining, so in normal operation there is nothing to park - but
 * "normally empty" is not a reason to lose data when it is not. */
static uint8_t gPend[64];
static uint16_t gPendLen;
static uint16_t gPendPos;

#if GW_LINK_DEDUP_ENABLE
/* Cache of the last reply, keyed by the SEQ it answered. See the discussion of
 * GW_LINK_DEDUP_ENABLE in gw_link_cfg.h: this is what makes an ESP32 retry safe
 * when the original reply was lost after the request had already executed. */
static uint8_t gLastSeq;   /* 0 = nothing cached */
static uint8_t gLastOp;
static uint16_t gLastTxLen;
static uint8_t gLastTx[GW_LINK_TX_FRAME_BYTES];
#endif

/* ==========================================================================
 * 2. Reply builder
 * ==========================================================================
 */

/** Payload area of the reply buffer. Handlers write their bytes here. */
static inline uint8_t* txPayload(void) { return &gTx[GW_RSP_HEADER_LEN]; }

/**
 * Forgets the cached reply.
 *
 * Called on every path that answers a frame we could not execute - a bad CRC or
 * an impossible length. Those replies still carry a SEQ, and caching one would
 * mean that a legitimate retry of that SEQ got the rejection back instead of
 * being executed. Rejections must never be sticky.
 */
static inline void invalidateDedup(void) {
#if GW_LINK_DEDUP_ENABLE
    gLastSeq = 0u;
    gLastTxLen = 0u;
#endif
}

/**
 * Frames and transmits one reply.
 *
 * Every reply in the protocol goes through here, including error replies, so
 * the framing exists in exactly one place. A reply is always sent - even for a
 * request that failed - because silence is the one answer the ESP32 cannot
 * distinguish from a dead cable. v1 had three paths that returned nothing at
 * all (an unimplemented MULWRITE, a short WRITE, and a wedged parser), and each
 * one cost the client a full timeout to discover.
 */
static void sendReply(uint8_t seq, uint8_t status, uint8_t reg, uint16_t payloadLen) {
    uint16_t crc;

    if (payloadLen > GW_MAX_PAYLOAD) {
        /* A handler asked for more than the buffer holds. Refuse rather than
         * transmit a frame whose CRC would be computed over a buffer overrun. */
        payloadLen = 0u;
        status = GW_ST_RANGE;
    }

    gTx[GW_RSP_OFF_SOF] = GW_SOF_RSP;
    gTx[GW_RSP_OFF_SEQ] = seq;
    gTx[GW_RSP_OFF_STATUS] = status;
    gTx[GW_RSP_OFF_REG] = reg;
    gwWr16(&gTx[GW_RSP_OFF_LEN], payloadLen);

    gTxLen = (uint16_t)(GW_RSP_HEADER_LEN + payloadLen);
    crc = crc16(gTx, gTxLen); /* CRC over header + payload, exactly as the ESP32 checks */
    gwWr16(&gTx[gTxLen], crc);
    gTxLen = (uint16_t)(gTxLen + GW_CRC_LEN);

    if (GwLinkPort_Send(gTx, gTxLen)) {
        gStats.txFrames++;
        GwImage_SysSetU32(GW_SYS_OFF_LINK_TX, gStats.txFrames);
    }

#if GW_LINK_DEDUP_ENABLE
    /* Cache for a possible retry of the same SEQ. SEQ 0 opts out. */
    if (seq != 0u) {
        gLastSeq = seq;
        gLastTxLen = gTxLen;
        memcpy(gLastTx, gTx, gTxLen);
    }
#endif
}

/* ==========================================================================
 * 3. Handlers
 * ==========================================================================
 * Contract for every handler:
 *   in   payload / payloadLen  the request's payload (may be empty)
 *        reg, off              the REG and OFF header fields
 *   out  *outLen               payload bytes written to txPayload()
 *   ret  GW_ST_*               OK, or why not
 *
 * A handler must not write more than GW_MAX_PAYLOAD bytes and must not block.
 * If a future handler ever needs to wait for something - a flash erase, a field
 * write - it returns GW_ST_BUSY or a token immediately and the caller polls.
 * Nothing in this file is ever allowed to wait for a bus.
 */

/**
 * ECHO - returns the payload unchanged.
 *
 * The first thing to run on a new cable, and the only command that proves the
 * link with no dependency on the image, the config, or any driver. If ECHO
 * round-trips 1 KB clean a thousand times, the wiring, baud rate, DMA setup,
 * framing and CRC are all correct, and anything still broken is above the
 * transport.
 */
static uint8_t handleEcho(const uint8_t* payload, uint16_t payloadLen, uint16_t* outLen) {
    if (payloadLen > GW_MAX_PAYLOAD) return GW_ST_BAD_LEN;
    if (payloadLen != 0u) memcpy(txPayload(), payload, payloadLen);
    *outLen = payloadLen;
    return GW_ST_OK;
}

/**
 * SYNC - identity and liveness handshake, run once at ESP32 boot and then
 * periodically as a heartbeat.
 *
 * stmUptimeMs going backwards is how the ESP32 detects that the industrial half
 * rebooted without either side needing a reset line or an event. That matters
 * because a reboot invalidates every timestamp the ESP32 holds: image
 * timestamps are milliseconds since STM32 boot, so after a restart an old
 * stamp would look impossibly fresh.
 *
 * The request payload (the ESP32's own uptime and wall clock) is accepted and
 * currently unused. It is in the protocol from the start because the industrial
 * half has no RTC by design, and the moment anything here wants wall-clock time
 * - an event ring entry, a config commit record - this is where it arrives.
 */
static uint8_t handleSync(const uint8_t* payload, uint16_t payloadLen, uint16_t* outLen) {
    GwSyncRsp rsp;

    /* Tolerate an empty request: a bring-up tool should be able to say SYNC
     * with nothing attached. Anything else than empty or exactly the right size
     * is a client bug worth reporting rather than guessing at. */
    if (payloadLen != 0u && payloadLen != sizeof(GwSyncReq)) return GW_ST_BAD_LEN;
    (void)payload;

    rsp.protoVersion = (uint8_t)GW_PROTO_VERSION;
    rsp.fwMajor = GW_FW_MAJOR;
    rsp.fwMinor = GW_FW_MINOR;
    rsp.fwPatch = GW_FW_PATCH;
    rsp.stmUptimeMs = HAL_GetTick();
    rsp.configCrc32 = GwImage_ConfigCrc32();
    rsp.configVersion = GwImage_ConfigVersion();

    /* Advertise only what is actually built. The ESP32 degrades on a missing
     * capability instead of erroring, so an older field unit keeps working when
     * the network half is upgraded first. */
    rsp.capabilities = 0u;

    memcpy(txPayload(), &rsp, sizeof(rsp));
    *outLen = (uint16_t)sizeof(rsp);
    return GW_ST_OK;
}

/**
 * RD_META - the layout of the image.
 *
 * REG = GW_REGION_ALL (0xFF) describes every region; any other value describes
 * that one region. The ESP32 calls this once at boot and derives every address
 * it will ever use from the answer, so no offset is hardcoded on the network
 * side and resizing a region stays a config change rather than a two-chip
 * firmware change.
 */
static uint8_t handleReadMeta(uint8_t reg, uint16_t* outLen) {
    GwMetaHeader hdr;
    uint8_t* out = txPayload();
    uint16_t used = 0u;
    uint8_t count = 0u;
    uint8_t i;

    hdr.protoVersion = (uint8_t)GW_PROTO_VERSION;
    hdr.imageBytes = GwImage_TotalBytes();
    hdr.configCrc32 = GwImage_ConfigCrc32();
    hdr.configVersion = GwImage_ConfigVersion();
    hdr.reserved = 0u;
    hdr.regions = 0u; /* patched below once the entries are counted */

    used = (uint16_t)sizeof(hdr);

    if (reg == GW_REGION_ALL) {
        for (i = 0u; i < GwImage_RegionCount(); i++) {
            const GwRegionDesc* d = GwImage_DescAt(i);
            if ((uint32_t)used + sizeof(GwRegionDesc) > GW_MAX_PAYLOAD) break;
            memcpy(out + used, d, sizeof(GwRegionDesc));
            used = (uint16_t)(used + sizeof(GwRegionDesc));
            count++;
        }
    } else {
        const GwRegionDesc* d = GwImage_Desc(reg);
        if (d == 0) return GW_ST_BAD_REGION;
        memcpy(out + used, d, sizeof(GwRegionDesc));
        used = (uint16_t)(used + sizeof(GwRegionDesc));
        count = 1u;
    }

    hdr.regions = count;
    memcpy(out, &hdr, sizeof(hdr));
    *outLen = used;
    return GW_ST_OK;
}

/**
 * RD_REGION - the workhorse. REG + OFF from the header, byte count from a
 * 2-byte payload.
 *
 * Note what this handler does NOT do: it does not touch a field bus, it does
 * not know what the bytes mean, and it cannot fail slowly. That is the entire
 * difference between v2 and v1, where the equivalent request ran a blocking
 * Modbus transaction and could cost 1.6 s.
 */
static uint8_t handleReadRegion(uint8_t reg, uint16_t off, const uint8_t* payload,
                                uint16_t payloadLen, uint16_t* outLen) {
    uint16_t count;

    if (payloadLen != sizeof(GwReadArgs)) return GW_ST_BAD_LEN;

    count = gwRd16(payload);
    if (count > GW_MAX_PAYLOAD) return GW_ST_BAD_LEN;

    /* GwImage_Read does the region and range checking. Doing it there rather
     * than here keeps every bounds check on the memory in one file. */
    {
        uint8_t st = GwImage_Read(reg, off, count, txPayload());
        if (st != GW_ST_OK) return st;
    }

    *outLen = count;
    return GW_ST_OK;
}

/**
 * WR_REGION - lets the network half fill a region it owns.
 *
 * Only regions flagged GW_REGF_WRITABLE accept this, which today means the
 * Modbus TCP mirror alone. Everything else belongs to an STM32 driver: letting
 * the radio half write the control half's process data would give away the
 * isolation the two-chip split exists to provide.
 *
 * This is NOT how a cloud setpoint reaches a field device. That is WR_CHANNEL
 * (P8): the write is queued, acknowledged as accepted, executed by the owning
 * driver on its next pass, and its outcome read back with RD_ACK. An inline
 * field write would put bus latency straight back onto the link, which is the
 * mistake v2 exists to undo.
 */
static uint8_t handleWriteRegion(uint8_t reg, uint16_t off, const uint8_t* payload,
                                 uint16_t payloadLen, uint16_t* outLen) {
    uint8_t st;

    if (payloadLen == 0u) return GW_ST_BAD_LEN;

    st = GwImage_Write(reg, off, payload, payloadLen);
    *outLen = 0u; /* an ack carries no data - the status byte is the answer */
    return st;
}

/* ==========================================================================
 * 4. Dispatch
 * ==========================================================================
 */

static void dispatch(void) {
    uint8_t seq = gFrame[GW_REQ_OFF_SEQ];
    uint8_t op = gFrame[GW_REQ_OFF_OP];
    uint8_t reg = gFrame[GW_REQ_OFF_REG];
    uint16_t off = gwRd16(&gFrame[GW_REQ_OFF_OFFSET]);
    uint16_t payloadLen = gwRd16(&gFrame[GW_REQ_OFF_LEN]);
    const uint8_t* payload = &gFrame[GW_REQ_HEADER_LEN];
    uint16_t outLen = 0u;
    uint8_t status;

    gStats.rxFrames++;
    gStats.lastRxMs = HAL_GetTick();
    GwImage_SysSetU32(GW_SYS_OFF_LINK_RX, gStats.rxFrames);

#if GW_LINK_DEDUP_ENABLE
    /*
     * Same SEQ as the last request we answered: this is a retry of a reply that
     * got lost, not a new request. Re-send the cached reply without executing
     * anything. For a read that only saves work; for a write it is the
     * difference between idempotent and applied twice.
     *
     * The opcode is compared as well, because a client that reuses a SEQ for a
     * different command has a bug, and answering its old reply would hide it.
     */
    if (seq != 0u && seq == gLastSeq && op == gLastOp && gLastTxLen != 0u) {
        if (GwLinkPort_Send(gLastTx, gLastTxLen)) {
            gStats.dupRetransmits++;
            gStats.txFrames++;
            GwImage_SysSetU32(GW_SYS_OFF_LINK_TX, gStats.txFrames);
        }
        return;
    }
    gLastOp = op;
#endif

    switch (op) {
        case GW_OP_ECHO:
            status = handleEcho(payload, payloadLen, &outLen);
            break;

        case GW_OP_SYNC:
            status = handleSync(payload, payloadLen, &outLen);
            break;

        case GW_OP_RD_META:
            status = handleReadMeta(reg, &outLen);
            break;

        case GW_OP_RD_REGION:
            status = handleReadRegion(reg, off, payload, payloadLen, &outLen);
            break;

        case GW_OP_WR_REGION:
            status = handleWriteRegion(reg, off, payload, payloadLen, &outLen);
            break;

        /* Reserved and answered honestly until their phase lands. Returning a
         * defined status instead of silence is what lets the ESP32 discover
         * what this firmware can do at runtime rather than from a version
         * number - see the capabilities field in SYNC. */
        case GW_OP_RD_DELTA:   /* P8 - change-only reads                       */
        case GW_OP_WR_CHANNEL: /* P8 - queued field writes                     */
        case GW_OP_RD_ACK:     /* P8 - outcome of a queued write               */
        case GW_OP_RD_EVENTS:  /* P8 - timestamped transition ring             */
        case GW_OP_CFG_BEGIN:  /* P5 - config download into flash              */
        case GW_OP_CFG_CHUNK:
        case GW_OP_CFG_COMMIT:
        case GW_OP_CFG_ABORT:
            status = GW_ST_NOT_IMPL;
            outLen = 0u;
            break;

        default:
            status = GW_ST_BAD_CMD;
            outLen = 0u;
            break;
    }

    sendReply(seq, status, reg, outLen);
}

/* ==========================================================================
 * 5. Parser
 * ==========================================================================
 */

static void resetParser(void) {
    gState = LP_HUNT;
    gFrameLen = 0u;
    gExpect = 0u;
}

/**
 * Feeds one received byte into the state machine.
 *
 * Everything that can go wrong with a byte stream is handled here and nowhere
 * else: a byte that is not a start byte, a length that cannot be right, a frame
 * that stops halfway, and a frame that arrives complete but corrupted.
 */
static void feedByte(uint8_t b) {
    switch (gState) {
        case LP_HUNT:
            if (b == GW_SOF_REQ) {
                gFrame[0] = b;
                gFrameLen = 1u;
                gFrameStartMs = HAL_GetTick();
                gState = LP_HEADER;
            } else {
                /* Noise, or the tail of a frame we gave up on. Counting these
                 * is worth the one instruction: a steadily climbing dropped
                 * count with no CRC errors means a baud-rate or ground problem,
                 * while dropped bytes in bursts alongside CRC errors mean
                 * electrical noise. The two have completely different fixes. */
                gStats.droppedBytes++;
                GwImage_SysSetU32(GW_SYS_OFF_LINK_DROPPED, gStats.droppedBytes);
            }
            break;

        case LP_HEADER:
            gFrame[gFrameLen++] = b;
            if (gFrameLen < GW_REQ_HEADER_LEN) break;

            {
                uint16_t len = gwRd16(&gFrame[GW_REQ_OFF_LEN]);

                /* The single most important check in this file. v1 had no
                 * length field and no bounds check at all, so a frame claiming
                 * a large count walked straight off the end of a 64-byte
                 * buffer - a remote memory corruption reachable from line
                 * noise. Here an impossible length is answered and discarded. */
                if (len > GW_MAX_PAYLOAD) {
                    gStats.frameErrors++;
                    GwImage_SysSetU32(GW_SYS_OFF_LINK_FRM_ERR, gStats.frameErrors);
                    sendReply(gFrame[GW_REQ_OFF_SEQ], GW_ST_BAD_LEN, gFrame[GW_REQ_OFF_REG], 0u);
                    invalidateDedup();
                    resetParser();
                    break;
                }

                gExpect = (uint16_t)(GW_REQ_HEADER_LEN + len + GW_CRC_LEN);
                gState = LP_BODY;

                /* A zero-payload request is complete as soon as its CRC lands,
                 * so fall through into LP_BODY on the next byte rather than
                 * special-casing it here. */
            }
            break;

        case LP_BODY:
            gFrame[gFrameLen++] = b;
            if (gFrameLen < gExpect) break;

            {
                uint16_t want = gwRd16(&gFrame[gExpect - GW_CRC_LEN]);
                uint16_t have = crc16(gFrame, (uint16_t)(gExpect - GW_CRC_LEN));

                if (want == have) {
                    dispatch();
                } else {
                    /* Answer the bad CRC rather than staying silent. The SEQ we
                     * echo may itself be corrupt, but the ESP32 checks the echo
                     * and will simply time out and retry if it does not match -
                     * which is strictly faster than waiting the full timeout
                     * every time a byte gets hit. */
                    gStats.crcErrors++;
                    GwImage_SysSetU32(GW_SYS_OFF_LINK_CRC_ERR, gStats.crcErrors);
                    sendReply(gFrame[GW_REQ_OFF_SEQ], GW_ST_BAD_CRC, gFrame[GW_REQ_OFF_REG], 0u);
                    invalidateDedup();
                }
                resetParser();
            }
            break;

        default:
            resetParser();
            break;
    }
}

/* ==========================================================================
 * 6. Public API
 * ==========================================================================
 */

bool GwLink_Init(UART_HandleTypeDef* huart) {
    gReady = false;
    memset(&gStats, 0, sizeof(gStats));
    resetParser();
    gPendLen = 0u;
    gPendPos = 0u;
#if GW_LINK_DEDUP_ENABLE
    gLastSeq = 0u;
    gLastOp = 0u;
    gLastTxLen = 0u;
#endif

    /* The image has to exist before the link can answer from it. Checking here
     * rather than in every handler means a failed image is one silent link
     * instead of a scatter of odd statuses. */
    if (GwImage_RegionCount() == 0u) return false;
    if (!GwLinkPort_Init(huart)) return false;

    gReady = true;
    return true;
}

void GwLink_Step(void) {
    if (!gReady) return;

    /* Keep the receive DMA alive first - if it has been aborted by an overrun,
     * everything below would read zero bytes forever. */
    GwLinkPort_Service();

    /*
     * Do not start parsing while a reply is still going out. The protocol is
     * strictly one transaction in flight, so there is nothing legitimate to
     * read yet, and the reply buffer that dispatch() would overwrite is being
     * read by the DMA right now. The bytes wait in the 2 KB DMA ring, which
     * holds 178 ms of traffic at 115200 baud - far longer than any reply takes.
     */
    if (GwLinkPort_TxBusy()) return;

    /*
     * An unfinished frame that stopped arriving is thrown away.
     *
     * In v1 a truncated request stayed in the buffer indefinitely and was
     * re-parsed - and re-executed - when the next unrelated byte turned up,
     * which is how a single lost byte could make the link answer the wrong
     * question for the rest of the run. With an explicit length and this
     * timeout, a truncated frame costs one timeout and nothing else.
     */
    if (gState != LP_HUNT && (uint32_t)(HAL_GetTick() - gFrameStartMs) > GW_LINK_FRAME_IDLE_MS) {
        gStats.frameErrors++;
        GwImage_SysSetU32(GW_SYS_OFF_LINK_FRM_ERR, gStats.frameErrors);
        resetParser();
    }

    /* Drain in chunks rather than byte by byte: one ring read of 64 bytes
     * instead of 64 reads of the DMA counter. Feeding stops the moment a reply
     * is queued, so a burst can never make us build a second reply on top of
     * one still in flight; whatever is left over is fed on the next pass. */
    for (;;) {
        if (gPendPos >= gPendLen) {
            gPendPos = 0u;
            gPendLen = GwLinkPort_Read(gPend, (uint16_t)sizeof(gPend));
            if (gPendLen == 0u) return;
        }

        while (gPendPos < gPendLen) {
            feedByte(gPend[gPendPos++]);
            if (GwLinkPort_TxBusy()) return;
        }
    }
}

const GwLinkStats* GwLink_Stats(void) { return &gStats; }

bool GwLink_IsReady(void) { return gReady; }

#else /* GW_LINK_ENABLE == 0 */

/* Link v2 is compiled out: USART3 belongs to the old 0xAA/0xBB handler. The
 * stubs keep main.c compiling unchanged so the switch really is one #define. */
static GwLinkStats gStatsDisabled;
bool GwLink_Init(UART_HandleTypeDef* huart) {
    (void)huart;
    return false;
}
void GwLink_Step(void) {}
const GwLinkStats* GwLink_Stats(void) { return &gStatsDisabled; }
bool GwLink_IsReady(void) { return false; }

#endif /* GW_LINK_ENABLE */
