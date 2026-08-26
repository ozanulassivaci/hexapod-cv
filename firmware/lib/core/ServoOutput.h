// Abstraction over driving or releasing one physical PCA9685 channel, so
// safety-relevant logic (watchdog, dwell guard) can be unit tested on the
// host against a fake, and only the real PCA9685 driver needs actual
// hardware -- same pattern as transport.RobotLink / MockRobotLink on the
// PC side.
#pragma once

#include <cstdint>

class ServoOutput {
public:
    virtual ~ServoOutput() = default;

    // Command channel (board, ch) to a raw pulse width. No limit checking
    // here -- that's GatedServoDriver's job, not this interface's.
    // Implementations write directly to hardware.
    virtual void setPulse(uint8_t board, uint8_t channel, uint16_t pulseUs) = 0;

    // Fully detach channel (board, ch): the PCA9685 full-off bit, not a
    // 0us pulse -- a zero-width pulse isn't "no signal", it's a signal a
    // servo could misinterpret. See SafeState.h for why release (not
    // hold) is this build's failsafe response.
    virtual void release(uint8_t board, uint8_t channel) = 0;
};
