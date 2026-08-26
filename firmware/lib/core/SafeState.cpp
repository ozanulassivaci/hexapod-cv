#include "SafeState.h"

#include "Config.h"

void enterSafeState(ServoOutput& output) {
    for (uint8_t ch = 0; ch < PCA9685_CHANNELS_PER_BOARD; ++ch) {
        output.release(PCA9685_ADDR_BOARD_A, ch);
        output.release(PCA9685_ADDR_BOARD_B, ch);
    }
}
