#include "process_image.h"

#include <string.h>

#include "log.h"

ProcessImage processImage;

static const char* TAG = "image";

namespace {
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
}  // namespace

const char* tagQualityName(TagQuality q) {
    switch (q) {
        case Q_GOOD: return "good";
        case Q_STALE: return "stale";
        case Q_COMM_FAIL: return "commFail";
        default: return "unknown";
    }
}

ProcessImage::ProcessImage() : _mutex(nullptr), _state(nullptr) {}

bool ProcessImage::begin() {
    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) return false;

    _state = (TagState*)calloc(tagCount(), sizeof(TagState));
    if (!_state) {
        LOGE(TAG, "out of memory for %u tags", (unsigned)tagCount());
        return false;
    }
    LOGI(TAG, "%u tags", (unsigned)tagCount());
    return true;
}

void ProcessImage::update(uint16_t i, uint16_t raw) {
    if (i >= tagCount() || !_state) return;
    const TagDef& d = tagTable()[i];

    const float base = d.isSigned ? (float)(int16_t)raw : (float)raw;

    Lock lock(_mutex);
    TagState& s = _state[i];
    s.raw = raw;
    s.value = base * d.scale + d.offset;
    s.updatedMs = millis();
    s.quality = Q_GOOD;
    s.failCount = 0;
    s.okCount++;
}

void ProcessImage::fail(uint16_t i, uint8_t stmStatus) {
    if (i >= tagCount() || !_state) return;

    Lock lock(_mutex);
    TagState& s = _state[i];
    s.errCount++;
    s.lastError = stmStatus;
    if (s.failCount < 0xFFFF) s.failCount++;
    if (s.failCount >= TAG_FAIL_LIMIT) {
        s.quality = Q_COMM_FAIL;
    } else if (s.quality == Q_GOOD) {
        s.quality = Q_STALE;
    }
}

TagState ProcessImage::state(uint16_t i) const {
    TagState out;
    memset(&out, 0, sizeof(out));
    if (i >= tagCount() || !_state) return out;
    Lock lock(_mutex);
    out = _state[i];
    return out;
}

void ProcessImage::snapshot(TagState* out) const {
    if (!out || !_state) return;
    Lock lock(_mutex);
    memcpy(out, _state, (size_t)tagCount() * sizeof(TagState));
}

void ProcessImage::ageTags() {
    if (!_state) return;
    const uint32_t now = millis();
    Lock lock(_mutex);
    for (uint16_t i = 0; i < tagCount(); ++i) {
        TagState& s = _state[i];
        if (s.quality == Q_GOOD && s.updatedMs != 0 && (now - s.updatedMs) > TAG_STALE_MS) {
            s.quality = Q_STALE;
        }
    }
}

uint16_t ProcessImage::goodCount() const {
    if (!_state) return 0;
    uint16_t n = 0;
    Lock lock(_mutex);
    for (uint16_t i = 0; i < tagCount(); ++i) {
        if (_state[i].quality == Q_GOOD) n++;
    }
    return n;
}
