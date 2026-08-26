#include <unity.h>

#include <set>
#include <utility>

#include "Config.h"
#include "SafeState.h"
#include "ServoOutput.h"

// Records every release()/setPulse() call instead of touching hardware --
// same role as MockRobotLink on the PC side, scoped to this one interface.
class FakeServoOutput : public ServoOutput {
public:
    void setPulse(uint8_t board, uint8_t channel, uint16_t pulseUs) override {
        pulseCalls.insert({board, channel});
    }
    void release(uint8_t board, uint8_t channel) override {
        releaseCalls.insert({board, channel});
    }

    std::set<std::pair<uint8_t, uint8_t>> pulseCalls;
    std::set<std::pair<uint8_t, uint8_t>> releaseCalls;
};

void setUp(void) {}
void tearDown(void) {}

void test_bench_mode_releases_every_channel_on_both_boards(void) {
    FakeServoOutput fake;
    enterSafeState(fake, SafetyMode::Bench);

    TEST_ASSERT_EQUAL(2 * PCA9685_CHANNELS_PER_BOARD, fake.releaseCalls.size());
    for (uint8_t ch = 0; ch < PCA9685_CHANNELS_PER_BOARD; ++ch) {
        TEST_ASSERT_TRUE(fake.releaseCalls.count({PCA9685_ADDR_BOARD_A, ch}) == 1);
        TEST_ASSERT_TRUE(fake.releaseCalls.count({PCA9685_ADDR_BOARD_B, ch}) == 1);
    }
}

void test_bench_mode_never_sets_a_pulse(void) {
    // Safe state must never drive a servo toward any position, only
    // release it -- see the design note in SafeState.h.
    FakeServoOutput fake;
    enterSafeState(fake, SafetyMode::Bench);
    TEST_ASSERT_EQUAL(0, fake.pulseCalls.size());
}

void test_assembled_mode_touches_nothing(void) {
    // Assembled mode's "hold" is the absence of gait/pulse calls in
    // main.cpp's control loop, not an action this function takes --
    // enterSafeState itself must not release or command anything.
    FakeServoOutput fake;
    enterSafeState(fake, SafetyMode::Assembled);
    TEST_ASSERT_EQUAL(0, fake.releaseCalls.size());
    TEST_ASSERT_EQUAL(0, fake.pulseCalls.size());
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_bench_mode_releases_every_channel_on_both_boards);
    RUN_TEST(test_bench_mode_never_sets_a_pulse);
    RUN_TEST(test_assembled_mode_touches_nothing);
    return UNITY_END();
}
