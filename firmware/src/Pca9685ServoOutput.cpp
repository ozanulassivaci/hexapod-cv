#include "Pca9685ServoOutput.h"

#include "Config.h"

Pca9685ServoOutput::Pca9685ServoOutput()
    : boardA_(PCA9685_ADDR_BOARD_A), boardB_(PCA9685_ADDR_BOARD_B) {}

bool Pca9685ServoOutput::begin() {
    bool okA = boardA_.begin();
    boardA_.setOscillatorFrequency(PCA9685_OSC_FREQ_BOARD_A_HZ);
    boardA_.setPWMFreq(PCA9685_PWM_FREQ_HZ);

    bool okB = boardB_.begin();
    boardB_.setOscillatorFrequency(PCA9685_OSC_FREQ_BOARD_B_HZ);
    boardB_.setPWMFreq(PCA9685_PWM_FREQ_HZ);

    // Defensive: explicitly release every channel on both boards rather
    // than trusting the power-on-reset default silently. Matches
    // SafeState's boot-time reasoning -- command nothing, but verify
    // "nothing" rather than assume it.
    for (uint8_t ch = 0; ch < PCA9685_CHANNELS_PER_BOARD; ++ch) {
        release(PCA9685_ADDR_BOARD_A, ch);
        release(PCA9685_ADDR_BOARD_B, ch);
    }

    return okA && okB;
}

Adafruit_PWMServoDriver* Pca9685ServoOutput::driverFor(uint8_t board) {
    if (board == PCA9685_ADDR_BOARD_A) return &boardA_;
    if (board == PCA9685_ADDR_BOARD_B) return &boardB_;
    return nullptr;
}

void Pca9685ServoOutput::setPulse(uint8_t board, uint8_t channel, uint16_t pulseUs) {
    Adafruit_PWMServoDriver* driver = driverFor(board);
    if (driver == nullptr || channel >= PCA9685_CHANNELS_PER_BOARD) {
        return;
    }
    driver->writeMicroseconds(channel, pulseUs);
}

void Pca9685ServoOutput::release(uint8_t board, uint8_t channel) {
    Adafruit_PWMServoDriver* driver = driverFor(board);
    if (driver == nullptr || channel >= PCA9685_CHANNELS_PER_BOARD) {
        return;
    }
    // The library's own documented convention for "signal fully off"
    // (see Adafruit_PWMServoDriver::setPin) -- off=4096 sets the PCA9685's
    // full-off bit, not a 0us pulse. A zero-width pulse isn't "no
    // signal", it's a signal a servo could misinterpret; this is the
    // electrically correct way to truly detach a channel.
    driver->setPWM(channel, 0, 4096);
}
