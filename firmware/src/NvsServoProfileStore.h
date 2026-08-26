// Real NVS-backed implementation of ServoProfileStore. One blob per
// servo_index (key "p0".."p17"), via the Arduino Preferences library
// (wraps ESP-IDF NVS). ~512 bytes per profile, well under any NVS blob
// size limit for 18 entries.
#pragma once

#include <Preferences.h>

#include "ServoProfileStore.h"

class NvsServoProfileStore : public ServoProfileStore {
public:
    // Opens the NVS_NAMESPACE namespace. Call once from setup().
    void begin();

    // Falls back to ServoProfile::defaultFor(servoIndex) if nothing has
    // been stored yet, or if the stored blob's size doesn't match the
    // current struct layout (e.g. after a firmware rebuild that changed
    // ServoProfile's fields) -- reads garbage never, a reset-to-default
    // instead. This doesn't protect against a same-size layout change
    // reinterpreting old fields incorrectly; there's no deployed hardware
    // yet to make that worth a full schema-versioning scheme for.
    ServoProfile get(uint8_t servoIndex) override;
    void set(uint8_t servoIndex, const ServoProfile& profile) override;

private:
    Preferences prefs_;
    static void keyFor(uint8_t servoIndex, char* buf, size_t bufLen);
};
