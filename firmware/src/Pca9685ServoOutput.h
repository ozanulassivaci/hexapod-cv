// Real hardware implementation of ServoOutput -- the only place in this
// firmware that talks to the PCA9685 boards directly. Everything else
// (watchdog, dwell guard, gated driver) works against the ServoOutput
// interface and is unit tested against a fake; this class is what makes
// that interface real, and can only be verified by compiling for the
// actual target (there is no host-side I2C to test against).
#pragma once

#include <Adafruit_PWMServoDriver.h>

#include "ServoOutput.h"

class Pca9685ServoOutput : public ServoOutput {
public:
    Pca9685ServoOutput();

    // Call once from setup(), after Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN).
    // Initializes both boards at PCA9685_PWM_FREQ_HZ using the per-board
    // oscillator frequency corrections from Config.h (see that file's
    // comment on why these must be measured, not trusted at nominal), and
    // explicitly releases every channel on both boards -- defensive: it
    // shouldn't matter given the chip's power-on-reset default, but
    // verifying it rather than assuming it costs nothing at boot.
    // Returns false if either board didn't ACK on the I2C bus.
    bool begin();

    void setPulse(uint8_t board, uint8_t channel, uint16_t pulseUs) override;
    void release(uint8_t board, uint8_t channel) override;

private:
    Adafruit_PWMServoDriver* driverFor(uint8_t board);

    Adafruit_PWMServoDriver boardA_;
    Adafruit_PWMServoDriver boardB_;
};
