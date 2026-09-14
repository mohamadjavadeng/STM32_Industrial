/**
 * localio_client.cpp - see localio_client.h.
 */
#include "localio_client.h"

#include "log.h"

static const char* TAG = "lio";

/**
 * Renders a 4-bit mask as "1=0 2=1 3=1 4=0" into `out`.
 *
 * Hex alone is not enough when the question is "did input 3 change" - counting
 * bits out of 0x0A at three in the morning is how a wiring fault gets blamed on
 * firmware. The caller supplies the buffer so this can be used twice in one
 * log line.
 */
static void renderBits(char* out, size_t cap, uint8_t mask, uint8_t count) {
    size_t used = 0;
    for (uint8_t i = 0; i < count && used + 5 < cap; ++i) {
        used += (size_t)snprintf(out + used, cap - used, "%s%u=%u", i ? " " : "",
                                 (unsigned)(i + 1), (unsigned)((mask >> i) & 1u));
    }
    if (cap) out[used < cap ? used : cap - 1] = '\0';
}

LocalIoClient localIo;

LocalIoClient::LocalIoClient()
    : _link(nullptr),
      _ready(false),
      _nextPollMs(0),
      _pollCount(0),
      _errorCount(0),
      _lastDeviceStatus(GW_ST_OK) {
    memset(&_map, 0, sizeof(_map));
    memset(&_state, 0, sizeof(_state));
    _state.quality = GW_Q_UNKNOWN;
}

bool LocalIoClient::begin(StmLinkV2& link) {
    _link = &link;
    _ready = false;

    StmV2Status st = _link->readMeta(_map);
    if (st != V2_OK) {
        LOGE(TAG, "RD_META failed: %s", stmV2StatusName(st));
        return false;
    }

    const GwRegionDesc* d = _map.find(GW_REGION_LOCAL_IO);
    if (d == nullptr) {
        LOGE(TAG, "STM32 has no LOCAL_IO region");
        return false;
    }

    /* The STM32 refuses to start its driver against a region too small for the
     * slot map, and so do we. Reading slot 17 out of a 4-slot region would come
     * back as GW_ST_RANGE on every poll, which is a confusing way to learn that
     * GW_SLOTS_LOCAL_IO was set too low. */
    if (d->slots < GW_LIO_SLOT_MAX) {
        LOGE(TAG, "LOCAL_IO has %u slots, need %u", (unsigned)d->slots,
             (unsigned)GW_LIO_SLOT_MAX);
        return false;
    }

    LOGI(TAG, "LOCAL_IO region found: %u slots, valueOff=%u stampOff=%u qualOff=%u "
              "cfgCrc=0x%08lX",
         (unsigned)d->slots, (unsigned)d->valueOff, (unsigned)d->stampOff,
         (unsigned)d->qualOff, (unsigned long)_map.configCrc32);

    /* _ready has to be true for refresh() to do anything, but it must not
     * survive a failed first read. Leaving it set after begin() returned false
     * was a real bug: the cloud client asks isReady() and would have published
     * an all-zero snapshot as though it had been measured. A gateway reporting
     * "every input is open" when it has never actually read one is worse than a
     * gateway reporting nothing. */
    _ready = true;
    if (!refresh()) {
        _ready = false;
        LOGE(TAG, "first LOCAL_IO read failed - not publishing IO until it succeeds");
        return false;
    }
    return true;
}

bool LocalIoClient::refresh() {
    if (!_ready || _link == nullptr) return false;

    /* One readSlots covers the whole map. It costs three RD_REGION round trips
     * - one per parallel block - regardless of how many slots are asked for,
     * which is why reading the unused slots between the relays and the inputs
     * is free and worth it for the simpler arithmetic. */
    GwSlot slots[GW_LIO_SLOT_MAX];
    StmV2Status st = _link->readSlots(_map, GW_REGION_LOCAL_IO, 0, GW_LIO_SLOT_MAX, slots);
    if (st != V2_OK) {
        ++_errorCount;
        LOGW(TAG, "poll failed: %s", stmV2StatusName(st));
        /* The snapshot keeps its last values and stops being valid. Publishing
         * the old mask as current is the exact failure the quality byte exists
         * to prevent. */
        _state.valid = false;
        _state.quality = GW_Q_COMM_FAIL;
        return false;
    }

    uint8_t worst = GW_Q_GOOD;
    /* Quality is reported as the worst across the slots that matter, because a
     * dashboard shows one indicator for "is this data trustworthy" and the
     * honest answer is the weakest link in the set. The unused slots between
     * the blocks are skipped - they are UNKNOWN forever by design and would
     * drag every snapshot down to UNKNOWN. */
    for (uint8_t i = 0; i < GW_LIO_DO_COUNT; ++i) {
        uint8_t q = slots[GW_LIO_DO_BASE + i].quality;
        if (q > worst) worst = q;
    }
    for (uint8_t i = 0; i < GW_LIO_DI_COUNT; ++i) {
        uint8_t q = slots[GW_LIO_DI_BASE + i].quality;
        if (q > worst) worst = q;
    }

    uint8_t newRelays = (uint8_t)(slots[GW_LIO_DO_WORD].raw & 0x0Fu);
    uint8_t newInputs = (uint8_t)(slots[GW_LIO_DI_WORD].raw & 0x0Fu);

    /* First successful read, or a bit moved. Logged at INFO because this is the
     * line someone stares at while poking a contact with a screwdriver - if it
     * does not appear, the problem is below this client and there is no point
     * looking at MQTT or the dashboard. */
    bool acquired = !_state.valid;  /* first read, or first after a failure */
    if (acquired || newInputs != _state.inputMask) {
        char before[24], after[24];
        renderBits(before, sizeof(before), _state.inputMask, GW_LIO_DI_COUNT);
        renderBits(after, sizeof(after), newInputs, GW_LIO_DI_COUNT);
        if (acquired) {
            LOGI(TAG, "inputs  = 0x%X  [%s]  (acquired)", (unsigned)newInputs, after);
        } else {
            LOGI(TAG, "inputs  0x%X -> 0x%X   [%s] -> [%s]", (unsigned)_state.inputMask,
                 (unsigned)newInputs, before, after);
        }
    }
    if (acquired || newRelays != _state.relayMask) {
        char after[24];
        renderBits(after, sizeof(after), newRelays, GW_LIO_DO_COUNT);
        LOGI(TAG, "relays  = 0x%X  [%s]%s", (unsigned)newRelays, after,
             acquired ? "  (acquired)" : "");
    }

    _state.relayMask = newRelays;
    _state.inputMask = newInputs;
    _state.stmStampMs = slots[GW_LIO_DI_WORD].stampMs;
    _state.sampledMs = millis();
    _state.quality = worst;
    _state.valid = true;
    ++_pollCount;

    /* Quality is separate from the bits. A slot the STM32 flagged stale still
     * has a value, and this is the only place that says so out loud. */
    if (worst != GW_Q_GOOD) {
        LOGW(TAG, "LOCAL_IO quality is %s - values are not trustworthy",
             gwQualityName(worst));
    }

    return true;
}

bool LocalIoClient::poll(uint32_t intervalMs) {
    if (!_ready) return false;

    uint32_t now = millis();
    if ((int32_t)(now - _nextPollMs) < 0) return false;
    _nextPollMs = now + intervalMs;

    return refresh();
}

StmV2Status LocalIoClient::setRelay(uint8_t index, bool on) {
    if (!_ready || _link == nullptr) return V2_ERR_NOT_READY;
    if (index >= GW_LIO_DO_COUNT) return V2_ERR_BAD_ARG;

    bool applied = false;
    StmV2Status st = _link->writeChannel(GW_REGION_LOCAL_IO, (uint16_t)(GW_LIO_DO_BASE + index),
                                         on ? 1u : 0u, GW_ENC_U32, &applied);
    _lastDeviceStatus = _link->deviceStatus();

    if (st != V2_OK) {
        LOGW(TAG, "relay %u -> %d refused: %s (dev %s)", (unsigned)(index + 1), (int)on,
             stmV2StatusName(st), gwStatusName(_lastDeviceStatus));
    } else {
        LOGI(TAG, "relay %u -> %d accepted (applied=%d)", (unsigned)(index + 1), (int)on,
             (int)applied);
    }
    return st;
}

StmV2Status LocalIoClient::setRelayMask(uint8_t mask) {
    if (!_ready || _link == nullptr) return V2_ERR_NOT_READY;
    if (mask > 0x0Fu) return V2_ERR_BAD_ARG;

    bool applied = false;
    StmV2Status st =
        _link->writeChannel(GW_REGION_LOCAL_IO, GW_LIO_DO_WORD, mask, GW_ENC_U32, &applied);
    _lastDeviceStatus = _link->deviceStatus();

    if (st != V2_OK) {
        LOGW(TAG, "relay mask 0x%X refused: %s (dev %s)", (unsigned)mask, stmV2StatusName(st),
             gwStatusName(_lastDeviceStatus));
    }
    return st;
}
