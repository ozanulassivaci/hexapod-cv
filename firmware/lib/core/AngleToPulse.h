// The one leaf function that converts a joint-angle-space servo command
// (already in the reference's servo-write convention -- see Kinematics.h's
// toServoDeg(), bipolar-90 for coxa/femur, zero-based-negated for tibia)
// into a PCA9685 pulse width. Nowhere else in gait/IK touches pulse units
// -- reference/ANALYSIS.md Section 7's "degrees everywhere except one
// leaf function".
//
// Clamps to the generic [BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US] envelope
// defensively before the result ever reaches GatedServoDriver, which then
// separately enforces the real per-servo bench-recorded limit -- two
// layers, no bypass, same enforcement point bench mode already uses.
#pragma once

#include <cstdint>

// neutralServoDeg/neutralPulseUs let coxa/femur (90 deg / NEUTRAL_PULSE_US)
// and tibia (0 deg / TIBIA_NEUTRAL_PULSE_US -- a placeholder pending real
// horn-mount verification, see Config.h) share one implementation despite
// having different neutral conventions (ANALYSIS.md Section 2). sign and
// offsetUs are the per-servo ServoProfile calibration values, applied
// around this joint's own neutral pulse, not a hardcoded global one.
uint16_t angleToPulseUs(float servoDeg, float neutralServoDeg, uint16_t neutralPulseUs, int8_t sign,
                         int16_t offsetUs);
