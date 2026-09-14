/**
 * stm_link_v2.cpp - implementation of the ESP32 side of link v2.
 *
 * Layout of this file:
 *   1. helpers            status names, float reinterpretation
 *   2. construction       begin(), buffers, mutex
 *   3. low-level I/O      byte reads with a deadline, draining
 *   4. one attempt        build, send, receive, validate
 *   5. transaction        retries and the mutex
 *   6. commands           echo / sync / meta / read / write / slots
 *
 * The interesting logic is all in (4) and (5): everything a link protocol needs
 * to get right is about what happens when a frame does not come back.
 */
#include "stm_link_v2.h"

#include <string.h>

#include "crc16_modbus.h"
#include "log.h"

static const char* TAG = "linkv2";

StmLinkV2 stmLinkV2;

/* ==========================================================================
 * 1. Helpers
 * ==========================================================================
 */

const char* stmV2StatusName(StmV2Status s) {
    switch (s) {
        case V2_OK: return "OK";
        case V2_ERR_TIMEOUT: return "TIMEOUT";
        case V2_ERR_CRC: return "CRC";
        case V2_ERR_FRAME: return "FRAME";
        case V2_ERR_SEQ: return "SEQ";
        case V2_ERR_DEVICE: return "DEVICE";
        case V2_ERR_BAD_ARG: return "BAD_ARG";
        case V2_ERR_OVERFLOW: return "OVERFLOW";
        case V2_ERR_NOT_READY: return "NOT_READY";
        default: return "?";
    }
}

float StmLinkV2::slotAsFloat(uint32_t raw) {
    float f;
    // memcpy, not a pointer cast: a cast is undefined behaviour under strict
    // aliasing and the compiler is entitled to reorder around it. The copy
    // costs nothing after optimisation.
    memcpy(&f, &raw, sizeof(f));
    return f;
}

const GwRegionDesc* GwImageMap::find(uint8_t regionId) const {
    for (uint8_t i = 0; i < regionCount && i < GW_REGION_COUNT; i++) {
        if (region[i].region == regionId) return &region[i];
    }
    return nullptr;
}

/* ==========================================================================
 * 2. Construction
 * ==========================================================================
 */

StmLinkV2::StmLinkV2()
    : _uart(nullptr), _ready(false), _seq(0), _deviceStatus(GW_ST_OK), _mutex(nullptr) {
    memset(&_stats, 0, sizeof(_stats));
}

bool StmLinkV2::begin(HardwareSerial& uart, int rxPin, int txPin, uint32_t baud) {
    _uart = &uart;

    if (_mutex == nullptr) {
        _mutex = xSemaphoreCreateRecursiveMutex();
        if (_mutex == nullptr) {
            LOGE(TAG, "mutex alloc failed");
            return false;
        }
    }

    // Must be called before begin() or the core ignores it.
    _uart->setRxBufferSize(STMV2_RX_BUFFER);
    _uart->begin(baud, SERIAL_8N1, rxPin, txPin);

    // Anything sitting in the UART now predates us: a half frame from a reset
    // mid-transaction, or noise from the pins floating before begin(). Feeding
    // it to the parser would cost one timeout to discover.
    drainRx();

    // Cheap insurance. The CRC is shared with the RS-485 side and with the
    // STM32; if this ever fails, every frame in the system is wrong and a
    // failure here is far easier to read than the symptoms downstream.
    if (!crc16ModbusSelfTest()) {
        LOGE(TAG, "CRC self-test failed - refusing to start");
        return false;
    }

    _ready = true;
    _seq = (uint8_t)(millis() & 0xFF);  // start somewhere arbitrary, see nextSeq()
    LOGI(TAG, "link v2 up: %lu baud, rx=%d tx=%d, proto %u.%u", (unsigned long)baud, rxPin, txPin,
         GW_PROTO_VERSION_MAJOR, GW_PROTO_VERSION_MINOR);
    return true;
}

/* ==========================================================================
 * 3. Low-level I/O
 * ==========================================================================
 */

/**
 * Sequence numbers run 1..255 and skip 0.
 *
 * 0 is reserved: the STM32 treats it as "do not deduplicate", so a caller that
 * genuinely wants a request executed twice can ask for it. Everything here
 * wants the opposite - a retry after a lost reply must not apply a write twice -
 * so normal traffic never uses 0.
 */
uint8_t StmLinkV2::nextSeq() {
    _seq++;
    if (_seq == 0) _seq = 1;
    return _seq;
}

void StmLinkV2::drainRx() {
    while (_uart->available()) {
        (void)_uart->read();
    }
}

/**
 * Reads one byte, yielding until the deadline.
 *
 * delay(1) rather than a spin: this runs on a FreeRTOS core that also has the
 * WiFi and TLS stacks to service, and a busy-wait here would starve them for
 * the whole timeout. Returns -1 when the deadline passes.
 */
int StmLinkV2::readByte(uint32_t deadlineMs) {
    for (;;) {
        if (_uart->available()) return _uart->read();
        if ((int32_t)(millis() - deadlineMs) >= 0) return -1;
        delay(1);
    }
}

/* ==========================================================================
 * 4. One attempt
 * ==========================================================================
 */

/**
 * Receives and validates one reply frame.
 *
 * The structure mirrors the STM32 parser exactly - hunt, header, body - which
 * is deliberate: two ends of a protocol that parse differently disagree at the
 * worst possible moment. Differences from the STM32 side are only about which
 * errors matter to a client:
 *
 *   - a byte that is not 0x5A while hunting is discarded silently. It is
 *     usually the tail of a reply to a request that already timed out.
 *   - a SEQ that is not ours is V2_ERR_SEQ, not a retry-worthy transport error.
 *     It means a late reply arrived; the caller retries and the stale frame is
 *     gone. In v1 this case was invisible and the late reply was consumed as
 *     the answer to the *next* request - wrong data, no error, anywhere.
 */
StmV2Status StmLinkV2::receiveFrame(uint8_t expectSeq, uint8_t* out, uint16_t outCap,
                                    uint16_t& outLen) {
    uint32_t startDeadline = millis() + STMV2_RSP_TIMEOUT_MS;
    int b;

    outLen = 0;

    // --- hunt for the start byte ------------------------------------------
    for (;;) {
        b = readByte(startDeadline);
        if (b < 0) {
            _stats.timeouts++;
            return V2_ERR_TIMEOUT;
        }
        if ((uint8_t)b == GW_SOF_RSP) break;
    }
    _rx[GW_RSP_OFF_SOF] = GW_SOF_RSP;

    // --- header ------------------------------------------------------------
    // Once a frame has started, the tighter inter-byte deadline applies: the
    // STM32 transmits a reply by DMA in one go, so a long gap mid-frame means
    // something is wrong rather than something is slow.
    for (uint16_t i = 1; i < GW_RSP_HEADER_LEN; i++) {
        b = readByte(millis() + STMV2_BYTE_TIMEOUT_MS);
        if (b < 0) {
            _stats.timeouts++;
            return V2_ERR_TIMEOUT;
        }
        _rx[i] = (uint8_t)b;
    }

    const uint8_t seq = _rx[GW_RSP_OFF_SEQ];
    const uint8_t status = _rx[GW_RSP_OFF_STATUS];
    const uint16_t len = gwRd16(&_rx[GW_RSP_OFF_LEN]);

    if (len > GW_MAX_PAYLOAD) {
        // Impossible length. Do not try to read that many bytes - that is how a
        // corrupted header turns into a multi-second stall, or worse on a side
        // that does not bounds-check.
        _stats.frameErrors++;
        return V2_ERR_FRAME;
    }

    // --- payload + CRC -----------------------------------------------------
    const uint16_t total = (uint16_t)(GW_RSP_HEADER_LEN + len + GW_CRC_LEN);
    for (uint16_t i = GW_RSP_HEADER_LEN; i < total; i++) {
        b = readByte(millis() + STMV2_BYTE_TIMEOUT_MS);
        if (b < 0) {
            _stats.timeouts++;
            return V2_ERR_TIMEOUT;
        }
        _rx[i] = (uint8_t)b;
    }

    if (!crc16ModbusCheck(_rx, total)) {
        _stats.crcErrors++;
        LOG_HEXDUMP(TAG, "bad-crc", _rx, total);
        return V2_ERR_CRC;
    }

    // --- validation --------------------------------------------------------
    if (expectSeq != 0 && seq != expectSeq) {
        _stats.seqErrors++;
        LOGW(TAG, "stale reply seq=%u want=%u - dropped", seq, expectSeq);
        return V2_ERR_SEQ;
    }

    _deviceStatus = status;

    if (len > outCap) {
        // The frame was fine; our buffer was not. Report it as ours, not as a
        // device error, so a caller cannot misread it as a field problem.
        _stats.frameErrors++;
        return V2_ERR_OVERFLOW;
    }

    if (len > 0 && out != nullptr) memcpy(out, &_rx[GW_RSP_HEADER_LEN], len);
    outLen = len;

    if (status != GW_ST_OK) {
        _stats.deviceErrors++;
        return V2_ERR_DEVICE;
    }

    return V2_OK;
}

StmV2Status StmLinkV2::attempt(uint8_t op, uint8_t reg, uint16_t off, const uint8_t* payload,
                               uint16_t payloadLen, uint8_t seq, uint8_t* out, uint16_t outCap,
                               uint16_t& outLen) {
    // --- build -------------------------------------------------------------
    _tx[GW_REQ_OFF_SOF] = GW_SOF_REQ;
    _tx[GW_REQ_OFF_SEQ] = seq;
    _tx[GW_REQ_OFF_OP] = op;
    _tx[GW_REQ_OFF_REG] = reg;
    gwWr16(&_tx[GW_REQ_OFF_OFFSET], off);
    gwWr16(&_tx[GW_REQ_OFF_LEN], payloadLen);
    if (payloadLen > 0 && payload != nullptr) {
        memcpy(&_tx[GW_REQ_HEADER_LEN], payload, payloadLen);
    }

    size_t frameLen = crc16ModbusAppend(_tx, GW_REQ_HEADER_LEN + payloadLen);

    // --- send --------------------------------------------------------------
    // Discard anything already in the RX buffer first. It can only be a late
    // reply or noise, and leaving it would be parsed as the start of ours.
    drainRx();

    uint32_t t0 = micros();
    _uart->write(_tx, frameLen);
    // flush(true) waits for the transmitter only. The no-argument flush() on
    // this core calls uart_flush_input() and THROWS AWAY the receive buffer -
    // which on a request/response link discards the reply that is already
    // arriving. This one-character difference cost real debugging time on v1.
    _uart->flush(true);

    LOG_HEXDUMP(TAG, "tx", _tx, frameLen);

    // --- receive -----------------------------------------------------------
    StmV2Status st = receiveFrame(seq, out, outCap, outLen);
    if (st == V2_OK) {
        _stats.lastRttUs = micros() - t0;
        if (_stats.lastRttUs > _stats.maxRttUs) _stats.maxRttUs = _stats.lastRttUs;
        _stats.lastOkMs = millis();
        _stats.replies++;
    }
    return st;
}

/* ==========================================================================
 * 5. Transaction
 * ==========================================================================
 */

StmV2Status StmLinkV2::transact(uint8_t op, uint8_t reg, uint16_t off, const uint8_t* payload,
                                uint16_t payloadLen, uint8_t* out, uint16_t outCap,
                                uint16_t& outLen) {
    if (!_ready) return V2_ERR_NOT_READY;
    if (payloadLen > GW_MAX_PAYLOAD) return V2_ERR_BAD_ARG;

    if (xSemaphoreTakeRecursive(_mutex,
                                pdMS_TO_TICKS(STMV2_RSP_TIMEOUT_MS * STMV2_MAX_ATTEMPTS + 100)) !=
        pdTRUE) {
        return V2_ERR_TIMEOUT;
    }

    // One SEQ for the whole transaction, reused by every retry.
    //
    // That is the point of the sequence number: if attempt 1 was executed but
    // its reply was lost, attempt 2 carries the same SEQ, and the STM32 answers
    // from its reply cache without executing anything again. A retry is then
    // idempotent for every opcode, including writes. Allocating a fresh SEQ per
    // attempt would make a retried write apply twice.
    const uint8_t seq = nextSeq();
    StmV2Status st = V2_ERR_TIMEOUT;

    for (uint8_t tryNo = 1; tryNo <= STMV2_MAX_ATTEMPTS; tryNo++) {
        _stats.requests++;
        st = attempt(op, reg, off, payload, payloadLen, seq, out, outCap, outLen);

        // A device error is a real answer: the STM32 understood and refused.
        // Retrying a BAD_REGION cannot help, and retrying hides the mistake.
        if (st == V2_OK || st == V2_ERR_DEVICE || st == V2_ERR_OVERFLOW) break;

        if (tryNo < STMV2_MAX_ATTEMPTS) {
            _stats.retries++;
            LOGW(TAG, "%s seq=%u attempt %u: %s", gwOpName(op), seq, tryNo, stmV2StatusName(st));
            delay(STMV2_GAP_MS + STMV2_RETRY_BASE_MS * tryNo);
        }
    }

    if (st != V2_OK && st != V2_ERR_DEVICE) {
        LOGE(TAG, "%s reg=%s off=%u failed: %s", gwOpName(op), gwRegionName(reg), off,
             stmV2StatusName(st));
    } else if (st == V2_ERR_DEVICE) {
        LOGW(TAG, "%s reg=%s off=%u refused: %s", gwOpName(op), gwRegionName(reg), off,
             gwStatusName(_deviceStatus));
    }

    xSemaphoreGiveRecursive(_mutex);
    return st;
}

StmV2Status StmLinkV2::sendRaw(const uint8_t* frame, uint16_t len, uint8_t expectSeq, uint8_t* out,
                               uint16_t outCap, uint16_t& outLen) {
    if (!_ready) return V2_ERR_NOT_READY;
    if (frame == nullptr || len == 0) return V2_ERR_BAD_ARG;

    if (xSemaphoreTakeRecursive(_mutex, pdMS_TO_TICKS(STMV2_RSP_TIMEOUT_MS + 100)) != pdTRUE) {
        return V2_ERR_TIMEOUT;
    }

    drainRx();
    _uart->write(frame, len);
    _uart->flush(true);
    LOG_HEXDUMP(TAG, "raw-tx", frame, len);

    _stats.requests++;
    StmV2Status st = receiveFrame(expectSeq, out, outCap, outLen);

    xSemaphoreGiveRecursive(_mutex);
    return st;
}

StmV2Stats StmLinkV2::stats() const { return _stats; }

void StmLinkV2::resetStats() { memset(&_stats, 0, sizeof(_stats)); }

/* ==========================================================================
 * 6. Commands
 * ==========================================================================
 */

StmV2Status StmLinkV2::echo(const uint8_t* data, uint16_t len, uint8_t* out, uint16_t outCap,
                            uint16_t& outLen) {
    if (len > GW_MAX_PAYLOAD) return V2_ERR_BAD_ARG;
    return transact(GW_OP_ECHO, 0, 0, data, len, out, outCap, outLen);
}

StmV2Status StmLinkV2::sync(GwSyncRsp& out) {
    GwSyncReq req;
    req.espUptimeMs = millis();
    req.espEpochSec = 0;  // filled once SNTP has resolved; see cloud_client.cpp

    uint8_t buf[sizeof(GwSyncRsp)];
    uint16_t len = 0;
    StmV2Status st = transact(GW_OP_SYNC, 0, 0, (const uint8_t*)&req, sizeof(req), buf,
                              (uint16_t)sizeof(buf), len);
    if (st != V2_OK) return st;
    if (len != sizeof(GwSyncRsp)) return V2_ERR_FRAME;

    memcpy(&out, buf, sizeof(out));

    // A major-version mismatch means the two chips disagree about the meaning
    // of every field that follows. Say so loudly here rather than letting the
    // caller publish nonsense - a wrong number with a plausible shape is the
    // hardest kind of fault to notice from a dashboard.
    if ((out.protoVersion >> 4) != GW_PROTO_VERSION_MAJOR) {
        LOGE(TAG, "protocol mismatch: STM32 v%u.%u, we are v%u.%u - reflash one of them",
             out.protoVersion >> 4, out.protoVersion & 0x0F, GW_PROTO_VERSION_MAJOR,
             GW_PROTO_VERSION_MINOR);
    }
    return V2_OK;
}

StmV2Status StmLinkV2::readMeta(GwImageMap& out) {
    uint8_t buf[sizeof(GwMetaHeader) + GW_REGION_COUNT * sizeof(GwRegionDesc)];
    uint16_t len = 0;

    memset(&out, 0, sizeof(out));

    StmV2Status st = transact(GW_OP_RD_META, GW_REGION_ALL, 0, nullptr, 0, buf,
                              (uint16_t)sizeof(buf), len);
    if (st != V2_OK) return st;
    if (len < sizeof(GwMetaHeader)) return V2_ERR_FRAME;

    GwMetaHeader hdr;
    memcpy(&hdr, buf, sizeof(hdr));

    if (hdr.regions > GW_REGION_COUNT) return V2_ERR_FRAME;
    if (len != sizeof(GwMetaHeader) + (size_t)hdr.regions * sizeof(GwRegionDesc)) {
        return V2_ERR_FRAME;
    }

    out.protoVersion = hdr.protoVersion;
    out.regionCount = hdr.regions;
    out.imageBytes = hdr.imageBytes;
    out.configCrc32 = hdr.configCrc32;
    out.configVersion = hdr.configVersion;

    for (uint8_t i = 0; i < hdr.regions; i++) {
        memcpy(&out.region[i], buf + sizeof(GwMetaHeader) + i * sizeof(GwRegionDesc),
               sizeof(GwRegionDesc));
    }

    // From here the config CRC should be compared against the one stamped into
    // this device's tag map, and publishing refused on a mismatch. The tag map
    // is still compiled in (tag_map.cpp), so there is nothing to compare yet -
    // wire this up in the same commit that introduces a downloaded tag map, and
    // not later. See the GwMetaHeader comment in shared/gw_model.h for what
    // goes wrong without it.
    return V2_OK;
}

StmV2Status StmLinkV2::readRegion(uint8_t region, uint16_t off, uint16_t len, uint8_t* out) {
    if (len == 0 || len > GW_MAX_PAYLOAD || out == nullptr) return V2_ERR_BAD_ARG;

    GwReadArgs args;
    args.count = len;

    uint16_t got = 0;
    StmV2Status st = transact(GW_OP_RD_REGION, region, off, (const uint8_t*)&args, sizeof(args),
                              out, len, got);
    if (st != V2_OK) return st;

    // A short read is a protocol violation, not a partial success. Treating it
    // as one would leave the tail of the caller's buffer holding whatever was
    // there before - stale values with no indication they are stale.
    if (got != len) return V2_ERR_FRAME;
    return V2_OK;
}

StmV2Status StmLinkV2::writeRegion(uint8_t region, uint16_t off, const uint8_t* data,
                                   uint16_t len) {
    if (len == 0 || len > GW_MAX_PAYLOAD || data == nullptr) return V2_ERR_BAD_ARG;

    uint16_t got = 0;
    return transact(GW_OP_WR_REGION, region, off, data, len, nullptr, 0, got);
}

StmV2Status StmLinkV2::writeChannel(uint8_t region, uint16_t slot, uint32_t value,
                                    uint8_t encoding, bool* applied, uint32_t* token) {
    GwWrChannelReq req;
    req.value = value;
    req.encoding = encoding;
    req.flags = 0;
    req.reserved = 0;

    uint8_t rsp[sizeof(GwWrChannelRsp)];
    uint16_t got = 0;

    StmV2Status st = transact(GW_OP_WR_CHANNEL, region, slot, (const uint8_t*)&req, sizeof(req),
                              rsp, sizeof(rsp), got);
    if (st != V2_OK) return st;

    // An OK status with the wrong payload size means the two sides disagree
    // about the structure, which is the one failure this protocol's version
    // check exists to make loud. Reading `applied` out of a short buffer would
    // turn it into a value that is occasionally right.
    if (got != sizeof(GwWrChannelRsp)) return V2_ERR_FRAME;

    GwWrChannelRsp parsed;
    memcpy(&parsed, rsp, sizeof(parsed));
    if (applied) *applied = (parsed.applied != 0);
    if (token) *token = parsed.token;
    return V2_OK;
}

StmV2Status StmLinkV2::readSlots(const GwImageMap& map, uint8_t region, uint16_t firstSlot,
                                 uint16_t count, GwSlot* out) {
    if (out == nullptr || count == 0) return V2_ERR_BAD_ARG;

    const GwRegionDesc* d = map.find(region);
    if (d == nullptr) return V2_ERR_BAD_ARG;
    if ((d->flags & GW_REGF_RAW) != 0) return V2_ERR_BAD_ARG;  // no slots here
    if ((uint32_t)firstSlot + count > d->slots) return V2_ERR_BAD_ARG;

    // One transaction per block. The 4-byte blocks dominate, so the practical
    // cap is GW_MAX_PAYLOAD / 4 slots per call - 256 at the default payload.
    if ((uint32_t)count * GW_SLOT_VALUE_BYTES > GW_MAX_PAYLOAD) return V2_ERR_BAD_ARG;

    // Hold the link across all three reads. The mutex is recursive, so the
    // transact() inside each readRegion() takes it again without deadlocking.
    // Two reasons to hold it: _scratch is reused three times, and the three
    // blocks of one snapshot should not be interleaved with another task's
    // traffic.
    if (xSemaphoreTakeRecursive(_mutex, pdMS_TO_TICKS(STMV2_RSP_TIMEOUT_MS * 6 + 100)) != pdTRUE) {
        return V2_ERR_TIMEOUT;
    }

    const uint16_t vLen = (uint16_t)(count * GW_SLOT_VALUE_BYTES);
    const uint16_t sLen = (uint16_t)(count * GW_SLOT_STAMP_BYTES);
    const uint16_t qLen = (uint16_t)(count * GW_SLOT_QUAL_BYTES);
    StmV2Status st;

    // Values, then straight into the caller's array - _scratch is reused for
    // the next block, so nothing is kept between steps.
    st = readRegion(region, (uint16_t)(d->valueOff + firstSlot * GW_SLOT_VALUE_BYTES), vLen,
                    _scratch);
    if (st == V2_OK) {
        for (uint16_t i = 0; i < count; i++) out[i].raw = gwRd32(&_scratch[i * GW_SLOT_VALUE_BYTES]);

        st = readRegion(region, (uint16_t)(d->stampOff + firstSlot * GW_SLOT_STAMP_BYTES), sLen,
                        _scratch);
    }
    if (st == V2_OK) {
        for (uint16_t i = 0; i < count; i++)
            out[i].stampMs = gwRd32(&_scratch[i * GW_SLOT_STAMP_BYTES]);

        st = readRegion(region, (uint16_t)(d->qualOff + firstSlot * GW_SLOT_QUAL_BYTES), qLen,
                        _scratch);
    }
    if (st == V2_OK) {
        for (uint16_t i = 0; i < count; i++) out[i].quality = _scratch[i];
    }

    xSemaphoreGiveRecursive(_mutex);
    return st;
}
