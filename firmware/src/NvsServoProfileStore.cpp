#include "NvsServoProfileStore.h"

#include <cstdio>

#include "Config.h"

void NvsServoProfileStore::begin() {
    prefs_.begin(NVS_NAMESPACE, /*readOnly=*/false);
}

void NvsServoProfileStore::keyFor(uint8_t servoIndex, char* buf, size_t bufLen) {
    snprintf(buf, bufLen, "p%u", servoIndex);
}

ServoProfile NvsServoProfileStore::get(uint8_t servoIndex) {
    char key[8];
    keyFor(servoIndex, key, sizeof(key));

    ServoProfile profile = ServoProfile::defaultFor(servoIndex);
    size_t stored = prefs_.getBytesLength(key);
    if (stored == sizeof(ServoProfile)) {
        prefs_.getBytes(key, &profile, sizeof(ServoProfile));
        profile.servoIndex = servoIndex;  // defensive: never trust a corrupt index back
    }
    return profile;
}

void NvsServoProfileStore::set(uint8_t servoIndex, const ServoProfile& profile) {
    char key[8];
    keyFor(servoIndex, key, sizeof(key));
    prefs_.putBytes(key, &profile, sizeof(ServoProfile));
}
