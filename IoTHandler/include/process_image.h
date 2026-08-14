/**
 * process_image.h - cached copy of every tag, with quality and age.
 *
 * This is the piece that decouples the cloud from the field bus. The poll task
 * writes here at its own cadence; the publisher reads a snapshot. Nothing in the
 * MQTT path ever waits on a serial transaction, so a dead RS-485 slave slows the
 * scan but never stalls a publish, and a TLS handshake never delays a poll.
 *
 * It also carries the quality flag that the link protocol cannot give us: the
 * STM32 answers every read with STATUS_OK even when its own Modbus transaction
 * failed (quirk Q5), so "did this value actually refresh" has to be tracked here
 * from timestamps and consecutive failure counts.
 */
#pragma once

#include <Arduino.h>

#include "config.h"
#include "tag_map.h"

enum TagQuality : uint8_t {
    Q_UNKNOWN = 0,   // never read since boot
    Q_GOOD = 1,      // fresh
    Q_STALE = 2,     // last read succeeded but too long ago
    Q_COMM_FAIL = 3  // repeated failures
};

const char* tagQualityName(TagQuality q);

struct TagState {
    uint16_t raw;
    float value;        // raw * scale + offset, sign-corrected
    uint32_t updatedMs; // millis() of last successful read, 0 = never
    uint8_t quality;
    uint16_t failCount;
    uint32_t okCount;
    uint32_t errCount;
    uint8_t lastError;  // StmStatus of the most recent failure
};

class ProcessImage {
   public:
    ProcessImage();

    bool begin();
    uint16_t size() const { return tagCount(); }
    const TagDef& def(uint16_t i) const { return tagTable()[i]; }

    /** Store a successful read. Applies scale/offset and sign handling. */
    void update(uint16_t i, uint16_t raw);

    /** Record a failed read; promotes to COMM_FAIL after TAG_FAIL_LIMIT. */
    void fail(uint16_t i, uint8_t stmStatus);

    /** Copy of one entry, consistent under concurrent updates. */
    TagState state(uint16_t i) const;

    /** Copies every entry into out[0..size()-1] under a single lock. */
    void snapshot(TagState* out) const;

    /** Re-evaluates GOOD -> STALE based on age. Call from the poll task. */
    void ageTags();

    uint16_t goodCount() const;

   private:
    mutable SemaphoreHandle_t _mutex;
    TagState* _state;
};

extern ProcessImage processImage;
