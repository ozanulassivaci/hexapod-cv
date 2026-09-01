#include "GatedServoDriver.h"

#include "Config.h"

GatedServoDriver::GatedServoDriver(ServoOutput& output, ServoProfileStore& profiles)
    : output_(output), profiles_(profiles) {}

bool GatedServoDriver::commandPulse(uint8_t board, uint8_t channel, uint16_t pulseUs,
                                     bool hasServoIndex, uint8_t servoIndex, float degFromNeutral,
                                     LimitMode limitMode, const char** reason) {
    if (hasServoIndex) {
        ServoProfile profile = profiles_.get(servoIndex);
        const float margin = (limitMode == LimitMode::Safe) ? SAFE_LIMIT_MARGIN_DEG : 0.0f;
        if (profile.hasMinDeg && degFromNeutral < profile.minDegFromNeutral + margin) {
            *reason = "pulse below recorded min for this servo";
            return false;
        }
        if (profile.hasMaxDeg && degFromNeutral > profile.maxDegFromNeutral - margin) {
            *reason = "pulse above recorded max for this servo";
            return false;
        }
    }
    output_.setPulse(board, channel, pulseUs);
    return true;
}

void GatedServoDriver::release(uint8_t board, uint8_t channel) {
    output_.release(board, channel);
}

void GatedServoDriver::releaseAll() {
    for (uint8_t ch = 0; ch < PCA9685_CHANNELS_PER_BOARD; ++ch) {
        output_.release(PCA9685_ADDR_BOARD_A, ch);
        output_.release(PCA9685_ADDR_BOARD_B, ch);
    }
}
