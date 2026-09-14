/**
 * stm_link_v2.h - ESP32 side of the inter-chip link, protocol version 2.
 * ---------------------------------------------------------------------------
 *
 * The counterpart of ModbusRTU_AWS/Core/Src/gw_link.c. Same wire contract, same
 * shared header (gw_model.h), opposite role: this side is the initiator and
 * always speaks first.
 *
 * WHY THE NETWORK HALF IS THE INITIATOR
 *   The ESP32 is unavailable at unpredictable times - WiFi roaming, TLS
 *   handshakes, broker backoff, OTA. If the STM32 pushed data, it would need a
 *   retry queue for a peer that vanishes without warning, on the chip that runs
 *   the control loop. As the initiator, the ESP32's stalls cost only its own
 *   data freshness, and the side that knows the publish cadence and the backlog
 *   is the side that sets the rate.
 *
 * WHAT REPLACED WHAT
 *   stm_link.h (v1, 0xAA/0xBB) asked the STM32 to run a live Modbus
 *   transaction and waited up to 1.6 s for it. It also carried eight documented
 *   firmware quirks, a resync hack for a parser that could wedge, and a
 *   read-back-after-write because a failed write still answered OK.
 *
 *   v2 asks for bytes out of a process image. The STM32 answers in
 *   microseconds, so the timeout drops to about 100 ms; quality arrives inside
 *   the payload instead of being guessed from timestamps; the sequence number
 *   removes the whole desync family; and none of the quirk workarounds exist,
 *   because none of the quirks do.
 *
 *   Both files can be built at once - they use different UART framing and
 *   different classes. Only one of them can own the serial port at a time,
 *   because only one protocol owns USART3 on the STM32 (GW_LINK_ENABLE there).
 *
 * THREADING
 *   Every public call is a complete, synchronous, retried transaction behind a
 *   mutex, so the poll task and a command handler can share one instance. No
 *   call busy-waits: every wait yields to the scheduler, which matters because
 *   the WiFi stack must keep running on the other core.
 *
 * CLOUD-AGNOSTIC ON PURPOSE
 *   Nothing in this file knows about AWS IoT, ThingsBoard or a bare MQTT
 *   broker. It produces raw values, timestamps and quality codes; which cloud
 *   they are shaped for is a decision made above it. That is what lets a
 *   multi-cloud back end be added without touching the link.
 */
#pragma once

#include <Arduino.h>

#include "config.h"
#include "gw_model.h"

/* ==========================================================================
 * Results
 * ==========================================================================
 * Two different failures, kept apart on purpose:
 *
 *   a TRANSPORT failure (timeout, CRC, framing) means we do not know what
 *   happened on the other side, and the caller should retry or degrade;
 *
 *   V2_ERR_DEVICE means the STM32 answered perfectly well and said no. The
 *   reason is in deviceStatus() as a GW_ST_* code. That is not a comms problem
 *   and retrying it will not help.
 */
enum StmV2Status : uint8_t {
    V2_OK = 0,
    V2_ERR_TIMEOUT,   // no reply within STMV2_RSP_TIMEOUT_MS
    V2_ERR_CRC,       // reply arrived complete but corrupted
    V2_ERR_FRAME,     // impossible header: bad SOF or bad length
    V2_ERR_SEQ,       // a reply to some other request - a late one, usually
    V2_ERR_DEVICE,    // STM32 answered a non-OK GW_ST_* - see deviceStatus()
    V2_ERR_BAD_ARG,   // caller asked for something out of range
    V2_ERR_OVERFLOW,  // reply larger than the buffer the caller supplied
    V2_ERR_NOT_READY, // begin() has not run
};

const char* stmV2StatusName(StmV2Status s);

struct StmV2Stats {
    uint32_t requests;
    uint32_t replies;
    uint32_t timeouts;
    uint32_t crcErrors;
    uint32_t frameErrors;
    uint32_t seqErrors;
    uint32_t deviceErrors;
    uint32_t retries;
    uint32_t lastRttUs;  // round trip of the last successful transaction
    uint32_t maxRttUs;   // worst since the last resetStats()
    uint32_t lastOkMs;   // millis() of the last success - link liveness
};

/**
 * The image layout as RD_META described it.
 *
 * Fetch this once after every STM32 reboot and derive every address from it.
 * Hardcoding an offset on this side would mean that resizing a region on the
 * STM32 silently shifts what every tag points at - the exact failure the
 * configCrc32 interlock exists to catch.
 */
struct GwImageMap {
    uint8_t protoVersion;
    uint8_t regionCount;
    uint16_t imageBytes;
    uint32_t configCrc32;
    uint16_t configVersion;
    GwRegionDesc region[GW_REGION_COUNT];

    /** Descriptor for a region id, or nullptr if the STM32 does not have it. */
    const GwRegionDesc* find(uint8_t regionId) const;
    bool valid() const { return regionCount > 0; }
};

/** One channel as it exists in the image: value, age and whether to trust it. */
struct GwSlot {
    uint32_t raw;      // the 4 bytes as stored; interpret per the tag map
    uint32_t stampMs;  // STM32 milliseconds since ITS boot, not ours
    uint8_t quality;   // GW_Q_*
};

class StmLinkV2 {
   public:
    StmLinkV2();

    /**
     * Opens the UART and clears any half-frame left in it.
     *
     * Defaults come from config.h. Serial1 is the STM32 link; Serial (UART0 on
     * GPIO43/44) is the debug console. Never swap them - logging into the link
     * would feed the STM32 parser garbage, and on this core the mistake is
     * quiet because both are HardwareSerial.
     */
    bool begin(HardwareSerial& uart = Serial1, int rxPin = PIN_STM_RX, int txPin = PIN_STM_TX,
               uint32_t baud = STM_LINK_BAUD);

    bool isReady() const { return _ready; }

    // ---- Commands ---------------------------------------------------------

    /**
     * ECHO - returns the payload unchanged. The first thing to run on new
     * hardware: it proves wiring, baud, framing and CRC with no dependency on
     * the image or any driver. If a 1 KB echo round-trips clean, the transport
     * is not the problem.
     */
    StmV2Status echo(const uint8_t* data, uint16_t len, uint8_t* out, uint16_t outCap,
                     uint16_t& outLen);

    /**
     * SYNC - identity and liveness.
     *
     * Watch stmUptimeMs: if it goes backwards, the STM32 rebooted, every image
     * timestamp you hold is meaningless (they count from ITS boot), and the map
     * should be re-read. This is how a reboot is detected without a reset line
     * and without the STM32 ever speaking first.
     */
    StmV2Status sync(GwSyncRsp& out);

    /** RD_META - the whole image layout in one transaction. */
    StmV2Status readMeta(GwImageMap& out);

    /** RD_REGION - len raw bytes from region+off into out. */
    StmV2Status readRegion(uint8_t region, uint16_t off, uint16_t len, uint8_t* out);

    /**
     * WR_REGION - push bytes into a region the STM32 has flagged writable.
     *
     * Today that is the Modbus TCP mirror only, and the intended user is a TCP
     * master running here on the network half. It is NOT the path for a cloud
     * setpoint: that is WR_CHANNEL, which queues the write for the owning
     * driver so that exactly one writer ever touches a slot.
     */
    StmV2Status writeRegion(uint8_t region, uint16_t off, const uint8_t* data, uint16_t len);

    /**
     * Reads `count` consecutive slots as value + timestamp + quality.
     *
     * Three RD_REGION transactions, one per parallel block, rather than one
     * transaction per slot - which is the reason the image stores the blocks
     * separately. Reading 100 channels costs three round trips instead of a
     * hundred.
     *
     * The three blocks are not sampled at the same instant, so a value can be a
     * few hundred microseconds older than its quality byte. That is inherent to
     * a lock-free image and is what the timestamps are for; if a strictly
     * consistent set is ever needed, read the SYS image sequence counter before
     * and after and retry on a change.
     */
    StmV2Status readSlots(const GwImageMap& map, uint8_t region, uint16_t firstSlot,
                          uint16_t count, GwSlot* out);

    // ---- Escape hatches ---------------------------------------------------

    /**
     * Generic transaction. Builds the frame, retries, checks SEQ and CRC.
     * Everything above is a thin wrapper around this; use it directly to try an
     * opcode that has no wrapper yet.
     */
    StmV2Status transact(uint8_t op, uint8_t reg, uint16_t off, const uint8_t* payload,
                         uint16_t payloadLen, uint8_t* out, uint16_t outCap, uint16_t& outLen);

    /**
     * Sends bytes exactly as given, with no framing and no CRC of ours, and
     * parses whatever comes back.
     *
     * This exists for the negative tests in the bring-up tool: deliberately
     * corrupt a CRC, claim an impossible length, use an unknown opcode. Error
     * handling that has never been exercised is decoration, and on a link this
     * is the only way to exercise it. Do not use it in production code.
     */
    StmV2Status sendRaw(const uint8_t* frame, uint16_t len, uint8_t expectSeq, uint8_t* out,
                        uint16_t outCap, uint16_t& outLen);

    /** GW_ST_* from the most recent reply. Meaningful after V2_ERR_DEVICE. */
    uint8_t deviceStatus() const { return _deviceStatus; }

    StmV2Stats stats() const;
    void resetStats();

    /** Reinterprets a 4-byte slot as a float. Bit pattern, not a conversion. */
    static float slotAsFloat(uint32_t raw);

   private:
    StmV2Status attempt(uint8_t op, uint8_t reg, uint16_t off, const uint8_t* payload,
                        uint16_t payloadLen, uint8_t seq, uint8_t* out, uint16_t outCap,
                        uint16_t& outLen);
    StmV2Status receiveFrame(uint8_t expectSeq, uint8_t* out, uint16_t outCap, uint16_t& outLen);
    int readByte(uint32_t deadlineMs);
    void drainRx();
    uint8_t nextSeq();

    HardwareSerial* _uart;
    bool _ready;
    uint8_t _seq;
    uint8_t _deviceStatus;

    // Recursive, so a multi-transaction operation such as readSlots() can hold
    // it across all three of its reads. That keeps the three parallel blocks of
    // one snapshot from being interleaved with another task's traffic, and lets
    // those callers share _scratch safely.
    SemaphoreHandle_t _mutex;

    StmV2Stats _stats;

    // One request and one reply in flight. The protocol allows no more, and a
    // second buffer would only make it possible to break that rule by accident.
    uint8_t _tx[GW_REQ_HEADER_LEN + GW_MAX_PAYLOAD + GW_CRC_LEN];
    uint8_t _rx[GW_RSP_HEADER_LEN + GW_MAX_PAYLOAD + GW_CRC_LEN];

    // Staging area for multi-block reads. A member rather than a local because
    // a kilobyte on a 4 KB task stack is not a risk worth taking.
    uint8_t _scratch[GW_MAX_PAYLOAD];
};

extern StmLinkV2 stmLinkV2;
