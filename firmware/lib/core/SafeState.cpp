#include "SafeState.h"

#include "Config.h"

void enterSafeState(ServoOutput& output, SafetyMode mode) {
    if (mode == SafetyMode::Bench) {
        for (uint8_t ch = 0; ch < PCA9685_CHANNELS_PER_BOARD; ++ch) {
            output.release(PCA9685_ADDR_BOARD_A, ch);
            output.release(PCA9685_ADDR_BOARD_B, ch);
        }
        return;
    }
    // Assembled: intentionally does nothing -- see the header comment.
    // Holding is the absence of the gait/pulse-emitting calls this
    // function's caller (main.cpp) skips while unsafe, not an action
    // this function takes.
}
