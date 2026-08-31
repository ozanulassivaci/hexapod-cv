#include "AngleToPulse.h"

#include <cmath>

#include "Config.h"

namespace {
constexpr float kUsPerDeg = (BENCH_PULSE_MAX_US - BENCH_PULSE_MIN_US) / 180.0f;
}

uint16_t angleToPulseUs(float servoDeg, float neutralServoDeg, uint16_t neutralPulseUs, int8_t sign,
                         int16_t offsetUs) {
    float pulse = static_cast<float>(neutralPulseUs) + sign * (servoDeg - neutralServoDeg) * kUsPerDeg +
                  static_cast<float>(offsetUs);
    if (pulse < BENCH_PULSE_MIN_US) pulse = BENCH_PULSE_MIN_US;
    if (pulse > BENCH_PULSE_MAX_US) pulse = BENCH_PULSE_MAX_US;
    return static_cast<uint16_t>(std::lround(pulse));
}

float pulseUsToDegFromNeutral(uint16_t pulseUs, uint16_t neutralPulseUs) {
    return (static_cast<float>(pulseUs) - static_cast<float>(neutralPulseUs)) / kUsPerDeg;
}
