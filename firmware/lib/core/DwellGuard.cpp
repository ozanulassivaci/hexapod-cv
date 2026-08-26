#include "DwellGuard.h"

DwellGuard::DwellGuard(float timeoutS) : timeoutS_(timeoutS) {}

int DwellGuard::indexFor(uint8_t board, uint8_t channel) {
    int boardOffset = (board == PCA9685_ADDR_BOARD_B) ? PCA9685_CHANNELS_PER_BOARD : 0;
    return boardOffset + channel;
}

bool DwellGuard::observe(uint8_t board, uint8_t channel, uint16_t pulseUs, float nowS) {
    int i = indexFor(board, channel);
    if (i < 0 || i >= static_cast<int>(2 * PCA9685_CHANNELS_PER_BOARD)) return false;

    if (pulseUs == NEUTRAL_PULSE_US) {
        awaySet_[i] = false;
        return false;
    }
    if (!awaySet_[i]) {
        awaySet_[i] = true;
        awaySinceS_[i] = nowS;
        return false;
    }
    return (nowS - awaySinceS_[i]) >= timeoutS_;
}

float DwellGuard::remainingS(uint8_t board, uint8_t channel, uint16_t pulseUs, float nowS) const {
    int i = indexFor(board, channel);
    if (i < 0 || i >= static_cast<int>(2 * PCA9685_CHANNELS_PER_BOARD)) return -1.0f;
    if (pulseUs == NEUTRAL_PULSE_US || !awaySet_[i]) return -1.0f;
    float elapsed = nowS - awaySinceS_[i];
    float remaining = timeoutS_ - elapsed;
    return remaining > 0.0f ? remaining : 0.0f;
}

void DwellGuard::reset(uint8_t board, uint8_t channel) {
    int i = indexFor(board, channel);
    if (i < 0 || i >= static_cast<int>(2 * PCA9685_CHANNELS_PER_BOARD)) return;
    awaySet_[i] = false;
}
