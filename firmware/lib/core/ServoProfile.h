// Mirrors transport/protocol.py's ServoProfile. Fixed-size POD so it can
// be stored as one NVS blob per servo -- see NvsServoProfileStore (src/,
// ESP32-only).
//
// min/maxPulseUs use an explicit has-flag, not a sentinel value like 0 --
// a servo that hasn't been bench-tested must be distinguishable from one
// whose limit was recorded as an actual value, same reasoning as the
// Python side's `int | None`.
#pragma once

#include <cstdint>
#include <cstring>

#include "Config.h"

struct ServoProfile {
    uint8_t servoIndex = 0;
    int16_t offsetUs = 0;
    int8_t sign = 1;
    bool hasMinPulse = false;
    uint16_t minPulseUs = 0;
    bool hasMaxPulse = false;
    uint16_t maxPulseUs = 0;
    char note[HEALTH_NOTE_MAX_LEN + 1] = {0};

    static ServoProfile defaultFor(uint8_t servoIndex) {
        ServoProfile profile;
        profile.servoIndex = servoIndex;
        return profile;
    }
};

// --- Pure merge operations ------------------------------------------------
//
// Each of these touches only the field(s) it's named for, preserving
// everything else in `existing`. This is how docs/protocol.md Section 7's
// "a write to one path must never clobber what the other has recorded"
// guarantee is actually implemented -- by construction, in one place,
// rather than by convention every call site has to remember separately.

// calibrate / write_offsets: offset_us/sign only.
ServoProfile mergeOffsetSign(const ServoProfile& existing, int16_t offsetUs, int8_t sign);

enum class LimitBound { Min, Max };

// record_limit: one bound only. Returns false (leaving `out` untouched)
// if applying this would make minPulseUs >= maxPulseUs for the resulting
// profile -- matches the rejection rule in docs/protocol.md's commands
// table. Does not itself range-check pulseUs against BENCH_PULSE_MIN_US/
// MAX_US -- that's the caller's job (protocol decode already does this
// before a value ever reaches here).
bool mergeLimit(const ServoProfile& existing, LimitBound bound, uint16_t pulseUs, ServoProfile& out);

// bench_health_note: note only. Truncates defensively to HEALTH_NOTE_MAX_LEN
// regardless of caller discipline -- protocol decode already enforces this
// bound, but this function doesn't trust that as its only line of defense.
ServoProfile mergeNote(const ServoProfile& existing, const char* note);
