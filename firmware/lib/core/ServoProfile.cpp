#include "ServoProfile.h"

ServoProfile mergeOffsetSign(const ServoProfile& existing, int16_t offsetUs, int8_t sign) {
    ServoProfile updated = existing;
    updated.offsetUs = offsetUs;
    updated.sign = sign;
    return updated;
}

bool mergeLimit(const ServoProfile& existing, LimitBound bound, uint16_t pulseUs, ServoProfile& out) {
    ServoProfile updated = existing;
    if (bound == LimitBound::Min) {
        if (updated.hasMaxPulse && pulseUs >= updated.maxPulseUs) {
            return false;
        }
        updated.hasMinPulse = true;
        updated.minPulseUs = pulseUs;
    } else {
        if (updated.hasMinPulse && pulseUs <= updated.minPulseUs) {
            return false;
        }
        updated.hasMaxPulse = true;
        updated.maxPulseUs = pulseUs;
    }
    out = updated;
    return true;
}

ServoProfile mergeNote(const ServoProfile& existing, const char* note) {
    ServoProfile updated = existing;
    std::strncpy(updated.note, note, HEALTH_NOTE_MAX_LEN);
    updated.note[HEALTH_NOTE_MAX_LEN] = '\0';
    return updated;
}
