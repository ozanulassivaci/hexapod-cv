#include "ServoProfile.h"

ServoProfile mergeOffsetSign(const ServoProfile& existing, int16_t offsetUs, int8_t sign) {
    ServoProfile updated = existing;
    updated.offsetUs = offsetUs;
    updated.sign = sign;
    return updated;
}

bool mergeLimit(const ServoProfile& existing, LimitBound bound, float degFromNeutral, ServoProfile& out) {
    ServoProfile updated = existing;
    if (bound == LimitBound::Min) {
        if (updated.hasMaxDeg && degFromNeutral >= updated.maxDegFromNeutral) {
            return false;
        }
        updated.hasMinDeg = true;
        updated.minDegFromNeutral = degFromNeutral;
    } else {
        if (updated.hasMinDeg && degFromNeutral <= updated.minDegFromNeutral) {
            return false;
        }
        updated.hasMaxDeg = true;
        updated.maxDegFromNeutral = degFromNeutral;
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
