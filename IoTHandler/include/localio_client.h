/**
 * localio_client.h - the network half's view of PD0..PD7.
 * ---------------------------------------------------------------------------
 *
 * Four relays and four inputs on the STM32, cached here so that the MQTT path
 * never waits on a serial transaction and the serial path never waits on TLS.
 * The counterpart is ModbusRTU_AWS/Core/Src/gw_localio.c.
 *
 * WHAT IT DOES NOT DO
 *   It does not know what a relay is wired to, and it does not hold a desired
 *   state of its own. A command goes to the STM32 and the next poll reports
 *   what the pins are actually doing. That round trip is the point: a dashboard
 *   switch that flips to ON the instant it is clicked is telling the operator
 *   about a request, not about a contactor, and the difference matters at
 *   exactly the moment something is broken.
 *
 * SLOT NUMBERS
 *   From shared/gw_model.h section 10, never from a constant of our own. The
 *   region's base offset comes from RD_META at runtime, so resizing LOCAL_IO on
 *   the STM32 needs no change here.
 */
#pragma once

#include <Arduino.h>

#include "stm_link_v2.h"

/** One snapshot of the local IO, taken at `sampledMs` on our clock. */
struct LocalIoState {
    uint8_t relayMask;  // bit 0 = relay 1, as the pins actually are
    uint8_t inputMask;  // bit 0 = input 1, debounced by the STM32
    uint8_t quality;    // worst GW_Q_* across the slots in this snapshot
    uint32_t sampledMs; // millis() when this snapshot was taken
    uint32_t stmStampMs;// STM32 tick stamped into the DI word slot
    bool valid;         // false until the first successful poll
};

class LocalIoClient {
   public:
    LocalIoClient();

    /**
     * Fetches the image layout and takes a first snapshot.
     *
     * Returns false when the STM32 has no LOCAL_IO region, when the region is
     * too small for the slot map, or when the link is not answering. A false
     * here means relays cannot be controlled at all, which the caller should
     * publish rather than hide - a dashboard with dead switches and no
     * explanation is the worst version of this failure.
     */
    bool begin(StmLinkV2& link);

    /** True once begin() has found a usable LOCAL_IO region. */
    bool isReady() const { return _ready; }

    /**
     * Re-reads the slots if `intervalMs` has elapsed. Cheap to call often.
     * Returns true when a fresh snapshot was taken.
     */
    bool poll(uint32_t intervalMs);

    /** Forces a read now, regardless of the interval. */
    bool refresh();

    LocalIoState state() const { return _state; }

    bool relay(uint8_t index) const { return (_state.relayMask >> index) & 1u; }
    bool input(uint8_t index) const { return (_state.inputMask >> index) & 1u; }

    /**
     * Commands one relay, 0-based. Returns V2_OK when the STM32 accepted it.
     *
     * Accepting is not the same as switching: the pin moves on the STM32's next
     * scan, and the state reported by this class only changes when a poll sees
     * it. A caller that wants to confirm should refresh() and compare, which is
     * what the RPC handler does.
     */
    StmV2Status setRelay(uint8_t index, bool on);

    /** Commands all four at once. bit 0 = relay 1. */
    StmV2Status setRelayMask(uint8_t mask);

    /** Reason the last command was refused, as a GW_ST_* code. */
    uint8_t lastDeviceStatus() const { return _lastDeviceStatus; }

    uint32_t pollCount() const { return _pollCount; }
    uint32_t errorCount() const { return _errorCount; }

    /** The layout as RD_META gave it, for diagnostics. */
    const GwImageMap& map() const { return _map; }

   private:
    StmLinkV2* _link;
    GwImageMap _map;
    LocalIoState _state;
    bool _ready;
    uint32_t _nextPollMs;
    uint32_t _pollCount;
    uint32_t _errorCount;
    uint8_t _lastDeviceStatus;
};

extern LocalIoClient localIo;
