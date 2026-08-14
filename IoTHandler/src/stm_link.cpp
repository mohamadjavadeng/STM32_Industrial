#include "stm_link.h"

#include <string.h>

#include "crc16_modbus.h"
#include "log.h"

static const char* TAG = "link";

StmLink stmLink;

namespace {

/** Scope guard for the link mutex. */
class Lock {
   public:
    explicit Lock(SemaphoreHandle_t h) : _h(h) {
        if (_h) xSemaphoreTake(_h, portMAX_DELAY);
    }
    ~Lock() {
        if (_h) xSemaphoreGive(_h);
    }

   private:
    SemaphoreHandle_t _h;
};

/** millis() difference that survives the 49-day wrap. */
inline bool elapsed(uint32_t deadlineMs) { return (int32_t)(millis() - deadlineMs) >= 0; }

}  // namespace

const char* stmStatusName(StmStatus s) {
    switch (s) {
        case STM_OK: return "ok";
        case STM_ERR_TIMEOUT: return "timeout";
        case STM_ERR_CRC: return "crc";
        case STM_ERR_FRAME: return "frame";
        case STM_ERR_DEVICE: return "device";
        case STM_ERR_BAD_ARG: return "bad-arg";
        case STM_ERR_UNSUPPORTED: return "unsupported";
        case STM_ERR_NOT_READY: return "not-ready";
        default: return "?";
    }
}

StmLink::StmLink() : _uart(nullptr), _ready(false), _mutex(nullptr) {
    memset(&_stats, 0, sizeof(_stats));
}

bool StmLink::begin(HardwareSerial& uart) {
    if (!crc16ModbusSelfTest()) {
        LOGE(TAG, "CRC self-test failed - this build could never agree with the STM32");
        return false;
    }

    _uart = &uart;
    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) return false;

    // Must be sized before begin(). A MULREAD reply can reach 205 bytes and the
    // task may be preempted mid-frame by the WiFi stack.
    _uart->setRxBufferSize(STM_RX_BUFFER);
    _uart->begin(STM_LINK_BAUD, SERIAL_8N1, PIN_STM_RX, PIN_STM_TX);

#if PIN_STM_EVENT >= 0
    pinMode(PIN_STM_EVENT, INPUT_PULLUP);
#endif
#if PIN_STM_RESET >= 0
    pinMode(PIN_STM_RESET, INPUT);  // released; drive LOW only to reset
#endif

    drainRx();
    _ready = true;
    LOGI(TAG, "up: %lu 8N1 rx=%d tx=%d quirks=%d", (unsigned long)STM_LINK_BAUD, PIN_STM_RX,
         PIN_STM_TX, STM_FW_QUIRKS);
    return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

StmStatus StmLink::readRegister(uint8_t logicalType, uint16_t addr, uint16_t& value) {
    uint8_t payload[4];  // a single read answers with exactly 2 payload bytes
    uint8_t len = 0;
    const StmStatus st = transaction(STM_CMD_READ, stmWireTypeForSingleRead(logicalType), addr,
                                     1, nullptr, 0, payload, sizeof(payload), len);
    if (st != STM_OK) return st;
    if (len < 2) return STM_ERR_FRAME;
    value = (uint16_t)((uint16_t)payload[0] << 8) | payload[1];
    return STM_OK;
}

StmStatus StmLink::readHolding(uint16_t addr, uint16_t& value) {
    return readRegister(STM_REG_HOLDING, addr, value);
}

StmStatus StmLink::readInput(uint16_t addr, uint16_t& value) {
    return readRegister(STM_REG_INPUT, addr, value);
}

StmStatus StmLink::readCoil(uint16_t addr, bool& value) {
    uint16_t raw = 0;
    const StmStatus st = readRegister(STM_REG_COIL, addr, raw);
    if (st == STM_OK) value = (raw != 0);
    return st;
}

StmStatus StmLink::readDiscrete(uint16_t addr, bool& value) {
    uint16_t raw = 0;
    const StmStatus st = readRegister(STM_REG_DISCRETE, addr, raw);
    if (st == STM_OK) value = (raw != 0);
    return st;
}

StmStatus StmLink::writeRegister(uint8_t logicalType, uint16_t addr, uint16_t value) {
    if (logicalType == STM_REG_INPUT || logicalType == STM_REG_DISCRETE) {
        return STM_ERR_BAD_ARG;  // read-only classes; STM32 drops them silently
    }
    // Quirk Q7: count must be exactly 2 or the STM32 never answers.
    uint8_t payload[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    uint8_t rsp[4];  // the write ack carries no payload
    uint8_t len = 0;
    return transaction(STM_CMD_WRITE, stmWireTypeForWrite(logicalType), addr, 2, payload, 2, rsp,
                       sizeof(rsp), len);
}

StmStatus StmLink::writeHolding(uint16_t addr, uint16_t value) {
    return writeRegister(STM_REG_HOLDING, addr, value);
}

StmStatus StmLink::writeCoil(uint16_t addr, bool value) {
    return writeRegister(STM_REG_COIL, addr, value ? 1 : 0);
}

StmStatus StmLink::writeRegisterVerified(uint8_t logicalType, uint16_t addr, uint16_t value) {
    const StmStatus st = writeRegister(logicalType, addr, value);
    if (st != STM_OK) return st;

    uint16_t readBack = 0;
    const StmStatus rd = readRegister(logicalType, addr, readBack);
    if (rd != STM_OK) return rd;

    if (logicalType == STM_REG_COIL) {
        return ((readBack != 0) == (value != 0)) ? STM_OK : STM_ERR_DEVICE;
    }
    return (readBack == value) ? STM_OK : STM_ERR_DEVICE;
}

StmStatus StmLink::readHoldingBlock(uint16_t addr, uint8_t regCount, uint16_t* out) {
    if (!out || regCount == 0 || regCount > STM_MAX_BLOCK_REGS) return STM_ERR_BAD_ARG;
    if (!stmBlockReadSupported(STM_REG_HOLDING)) return STM_ERR_UNSUPPORTED;

    // Quirk Q2: the wire count is the payload byte count, and the STM32 turns
    // that same number into the register quantity it asks the field slave for.
    const uint8_t wireCount = (uint8_t)(regCount * 2);

    uint8_t payload[STM_MAX_BLOCK_REGS * 2];
    uint8_t len = 0;
    const StmStatus st = transaction(STM_CMD_MULREAD, STM_REG_HOLDING, addr, wireCount, nullptr, 0,
                                     payload, sizeof(payload), len);
    if (st != STM_OK) return st;
    if (len < wireCount) return STM_ERR_FRAME;

    for (uint8_t i = 0; i < regCount; ++i) {
        out[i] = (uint16_t)((uint16_t)payload[i * 2] << 8) | payload[i * 2 + 1];
    }
    return STM_OK;
}

StmStatus StmLink::ping(uint16_t probeAddr) {
    uint16_t scratch = 0;
    return readHolding(probeAddr, scratch);
}

StmLinkStats StmLink::stats() const {
    Lock lock(_mutex);
    return _stats;
}

void StmLink::resetStats() {
    Lock lock(_mutex);
    const uint32_t keepOk = _stats.lastOkMs;
    memset(&_stats, 0, sizeof(_stats));
    _stats.lastOkMs = keepOk;
}

// ---------------------------------------------------------------------------
// Transaction core
// ---------------------------------------------------------------------------

StmStatus StmLink::transaction(uint8_t cmd, uint8_t wireType, uint16_t addr, uint8_t count,
                               const uint8_t* payload, uint8_t payloadLen, uint8_t* out,
                               uint8_t outCap, uint8_t& outLen) {
    if (!_ready) return STM_ERR_NOT_READY;
    if (payloadLen > STM_MAX_REQUEST_LEN) return STM_ERR_BAD_ARG;

    Lock lock(_mutex);

    StmStatus st = STM_ERR_TIMEOUT;
    for (uint8_t tryNo = 1; tryNo <= STM_MAX_ATTEMPTS; ++tryNo) {
        if (tryNo > 1) _stats.retries++;

        st = attempt(cmd, wireType, addr, count, payload, payloadLen, out, outCap, outLen);
        if (st == STM_OK) return st;

        // Caller error - retrying cannot help.
        if (st == STM_ERR_BAD_ARG || st == STM_ERR_UNSUPPORTED) return st;

        // Only MULREAD can leave the STM32 parser holding our frame (quirk Q4):
        // its failure path `continue`s without resetting rxIndex, while READ and
        // WRITE always fall through to the reset. So a failed MULREAD *must* be
        // flushed, including after the final attempt - otherwise every later byte
        // re-triggers the stuck request, and since rxIndex keeps incrementing it
        // eventually walks off the end of the STM32's 64-byte buffer (Q8).
        //
        // A failed READ/WRITE needs no resync: if our request was truncated the
        // STM32 self-heals on the next frame by failing its CRC check, answering
        // STATUS_ERROR and resetting. Skipping the resync there saves ~120 ms per
        // attempt, which matters because it is once per dead tag per scan.
        if (cmd == STM_CMD_MULREAD) resync();
        if (tryNo < STM_MAX_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)STM_RETRY_BASE_MS * tryNo));
        }
    }
    return st;
}

StmStatus StmLink::attempt(uint8_t cmd, uint8_t wireType, uint16_t addr, uint8_t count,
                           const uint8_t* payload, uint8_t payloadLen, uint8_t* out,
                           uint8_t outCap, uint8_t& outLen) {
    _tx[0] = STM_START_REQ;
    _tx[1] = cmd;
    _tx[2] = wireType;
    _tx[3] = (uint8_t)(addr >> 8);
    _tx[4] = (uint8_t)(addr & 0xFF);
    _tx[5] = count;
    if (payloadLen && payload) memcpy(&_tx[STM_REQ_HEADER_LEN], payload, payloadLen);
    const size_t frameLen = crc16ModbusAppend(_tx, (size_t)STM_REQ_HEADER_LEN + payloadLen);

    // Discard anything left over from a previous timed-out transaction so a late
    // reply is never mistaken for this one's.
    drainRx();

    const uint32_t startedMs = millis();
    _uart->write(_tx, frameLen);
    // flush(true) = TX only. The no-argument flush() also calls
    // uart_flush_input(), which would throw away the reply we are about to wait
    // for if the STM32 answers while we are still draining.
    _uart->flush(true);
    _stats.requests++;
    LOG_HEXDUMP(TAG, "tx", _tx, frameLen);

    const StmStatus st = receive(out, outCap, outLen);
    if (st == STM_OK) {
        _stats.replies++;
        _stats.lastOkMs = millis();
        _stats.lastRoundTripMs = millis() - startedMs;
    }

    // Give the STM32 main loop a moment before the next frame: it services the
    // link by polling single bytes and has just finished a blocking RS-485
    // transaction.
    vTaskDelay(pdMS_TO_TICKS(STM_GAP_MS));
    return st;
}

StmStatus StmLink::receive(uint8_t* out, uint8_t outCap, uint8_t& outLen) {
    outLen = 0;

    // 1. Hunt for the response start byte. Anything before it is line noise or
    //    the tail of a frame we already gave up on.
    const uint32_t overallDeadline = millis() + STM_RSP_TIMEOUT_MS;
    int b;
    do {
        b = readByte(overallDeadline);
        if (b < 0) {
            _stats.timeouts++;
            return STM_ERR_TIMEOUT;
        }
    } while (b != STM_START_RSP);

    _rx[0] = STM_START_RSP;

    // 2. status + payload length. Responses carry an explicit length, which is
    //    what lets one parser handle 5-byte acks, 7-byte reads and 205-byte
    //    block reads.
    for (uint8_t i = 1; i < STM_RSP_HEADER_LEN; ++i) {
        b = readByte(millis() + STM_BYTE_TIMEOUT_MS);
        if (b < 0) {
            _stats.timeouts++;
            return STM_ERR_TIMEOUT;
        }
        _rx[i] = (uint8_t)b;
    }

    const uint8_t status = _rx[1];
    const uint8_t payloadLen = _rx[2];
    const size_t total = (size_t)STM_RSP_HEADER_LEN + payloadLen + STM_CRC_LEN;
    if (total > sizeof(_rx)) {
        _stats.frameErrors++;
        return STM_ERR_FRAME;
    }

    // 3. payload + CRC.
    for (size_t i = STM_RSP_HEADER_LEN; i < total; ++i) {
        b = readByte(millis() + STM_BYTE_TIMEOUT_MS);
        if (b < 0) {
            _stats.timeouts++;
            return STM_ERR_TIMEOUT;
        }
        _rx[i] = (uint8_t)b;
    }

    if (!crc16ModbusCheck(_rx, total)) {
        _stats.crcErrors++;
        return STM_ERR_CRC;
    }

    if (status != STM_STATUS_OK) {
        // The STM32 only ever sends STATUS_ERROR for a CRC mismatch on our
        // request, so this means the frame we sent got corrupted on the way in.
        _stats.deviceErrors++;
        return STM_ERR_DEVICE;
    }

    const uint8_t copyLen = payloadLen < outCap ? payloadLen : outCap;
    if (copyLen && out) memcpy(out, &_rx[STM_RSP_HEADER_LEN], copyLen);
    outLen = copyLen;
    return STM_OK;
}

int StmLink::readByte(uint32_t deadlineMs) {
    for (;;) {
        const int c = _uart->read();
        if (c >= 0) return c;
        if (elapsed(deadlineMs)) return -1;
        vTaskDelay(1);  // 1 ms tick; the driver ring buffer absorbs the burst
    }
}

void StmLink::drainRx() {
    while (_uart->read() >= 0) {
    }
}

void StmLink::drainUntilQuiet(uint32_t quietMs, uint32_t capMs) {
    const uint32_t hardStop = millis() + capMs;
    uint32_t lastByteMs = millis();
    while (!elapsed(hardStop)) {
        if (_uart->read() >= 0) {
            lastByteMs = millis();
            continue;
        }
        if (millis() - lastByteMs >= quietMs) return;
        vTaskDelay(1);
    }
}

void StmLink::resync() {
    if (!_ready) return;

    // Quirk Q4: after a failed MULREAD the STM32 still holds our frame with a
    // non-zero rxIndex. One extra byte pushes it past its length check, so it
    // re-parses and re-executes the stale request and then clears rxIndex. Only
    // the read-only MULREAD path can get stuck like this, so the replay is safe.
    // We then swallow whatever that replay emits.
    _uart->write((uint8_t)0x00);
    _uart->flush(true);
    drainUntilQuiet(STM_RESYNC_QUIET_MS, STM_RESYNC_CAP_MS);
    _stats.resyncs++;
    LOGD(TAG, "resync (#%lu)", (unsigned long)_stats.resyncs);
}
