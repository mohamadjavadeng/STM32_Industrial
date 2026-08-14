/**
 * stm_link.h - transaction master for the UART link to the STM32.
 *
 * The ESP32 is the initiator on this link and the STM32 is a pure responder.
 * That split is deliberate: the STM32 owns the process (it is the Modbus master
 * on RS-485 and runs the IO), while the radio side reboots, reconnects and does
 * TLS handshakes at unpredictable times. Making the data owner a responder means
 * a stalled or crashed ESP32 can never stall the control loop, and the side that
 * knows the cloud cadence and its own backlog is the side that sets the rate.
 *
 * Every public method is a complete, synchronous, retried transaction guarded by
 * a mutex, so several tasks may share one instance safely. Nothing here blocks
 * on a plain delay(): waits yield to the scheduler.
 */
#pragma once

#include <Arduino.h>

#include "config.h"
#include "stm_protocol.h"

enum StmStatus : uint8_t {
    STM_OK = 0,
    STM_ERR_TIMEOUT,      // no reply within STM_RSP_TIMEOUT_MS
    STM_ERR_CRC,          // reply arrived but its CRC is wrong
    STM_ERR_FRAME,        // reply malformed / impossible length
    STM_ERR_DEVICE,       // STM32 answered STATUS_ERROR (it rejected our CRC)
    STM_ERR_BAD_ARG,      // caller asked for something out of range
    STM_ERR_UNSUPPORTED,  // not possible against this STM32 firmware
    STM_ERR_NOT_READY,    // begin() not called
};

const char* stmStatusName(StmStatus s);

struct StmLinkStats {
    uint32_t requests;
    uint32_t replies;
    uint32_t timeouts;
    uint32_t crcErrors;
    uint32_t frameErrors;
    uint32_t deviceErrors;
    uint32_t retries;
    uint32_t resyncs;
    uint32_t lastOkMs;
    uint32_t lastRoundTripMs;
};

class StmLink {
   public:
    StmLink();

    bool begin(HardwareSerial& uart = Serial1);
    bool isReady() const { return _ready; }

    // ---- Single register / bit access -------------------------------------
    StmStatus readHolding(uint16_t addr, uint16_t& value);
    StmStatus readInput(uint16_t addr, uint16_t& value);
    StmStatus readCoil(uint16_t addr, bool& value);
    StmStatus readDiscrete(uint16_t addr, bool& value);

    StmStatus writeHolding(uint16_t addr, uint16_t value);
    StmStatus writeCoil(uint16_t addr, bool value);

    /** Dispatches on a logical STM_REG_* class. */
    StmStatus readRegister(uint8_t logicalType, uint16_t addr, uint16_t& value);
    StmStatus writeRegister(uint8_t logicalType, uint16_t addr, uint16_t value);

    /**
     * Write then read back and compare. The STM32 always answers a write with
     * STATUS_OK even if the field write failed (quirk Q5), so this is the only
     * way to be sure. Returns STM_ERR_DEVICE when the read-back disagrees.
     */
    StmStatus writeRegisterVerified(uint8_t logicalType, uint16_t addr, uint16_t value);

    /**
     * Block read of consecutive holding registers, cmd MULREAD.
     *
     * Beware quirk Q2: to return regCount registers the STM32 asks the RS-485
     * slave for 2*regCount registers. If the field device does not have that
     * many contiguous readable registers it will answer with an exception and
     * this fails. Single reads are safer; use blocks only when you know the
     * slave's map. regCount <= STM_MAX_BLOCK_REGS.
     */
    StmStatus readHoldingBlock(uint16_t addr, uint8_t regCount, uint16_t* out);

    /** Feeds the STM32 parser one byte to clear a stuck frame (quirk Q4). */
    void resync();

    /** Cheap liveness probe: single read that we do not care about the value of. */
    StmStatus ping(uint16_t probeAddr);

    StmLinkStats stats() const;
    void resetStats();

   private:
    StmStatus transaction(uint8_t cmd, uint8_t wireType, uint16_t addr, uint8_t count,
                          const uint8_t* payload, uint8_t payloadLen, uint8_t* out,
                          uint8_t outCap, uint8_t& outLen);
    StmStatus attempt(uint8_t cmd, uint8_t wireType, uint16_t addr, uint8_t count,
                      const uint8_t* payload, uint8_t payloadLen, uint8_t* out,
                      uint8_t outCap, uint8_t& outLen);
    StmStatus receive(uint8_t* out, uint8_t outCap, uint8_t& outLen);
    int readByte(uint32_t deadlineMs);
    void drainRx();
    void drainUntilQuiet(uint32_t quietMs, uint32_t capMs);

    HardwareSerial* _uart;
    bool _ready;
    SemaphoreHandle_t _mutex;
    StmLinkStats _stats;
    uint8_t _rx[STM_RSP_HEADER_LEN + STM_MAX_RSP_PAYLOAD + STM_CRC_LEN];
    uint8_t _tx[STM_REQ_HEADER_LEN + STM_MAX_REQUEST_LEN + STM_CRC_LEN];
};

extern StmLink stmLink;
