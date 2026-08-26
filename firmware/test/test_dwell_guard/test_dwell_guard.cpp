#include <unity.h>

#include "Config.h"
#include "DwellGuard.h"

void setUp(void) {}
void tearDown(void) {}

void test_neutral_never_trips(void) {
    DwellGuard guard(8.0f);
    TEST_ASSERT_FALSE(guard.observe(PCA9685_ADDR_BOARD_A, 0, NEUTRAL_PULSE_US, 0.0f));
    TEST_ASSERT_FALSE(guard.observe(PCA9685_ADDR_BOARD_A, 0, NEUTRAL_PULSE_US, 1000.0f));
}

void test_trips_at_timeout(void) {
    DwellGuard guard(8.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 0.0f);
    TEST_ASSERT_FALSE(guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 7.9f));
    TEST_ASSERT_TRUE(guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 8.0f));
}

void test_channels_are_independent(void) {
    // The gap this exists to close: switching which channel is "selected"
    // must not abandon a different channel's actual physical state.
    DwellGuard guard(8.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 0.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 1, NEUTRAL_PULSE_US, 5.0f);  // a different channel, at neutral
    TEST_ASSERT_TRUE(guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 8.0f));   // channel 0 still trips on schedule
    TEST_ASSERT_FALSE(guard.observe(PCA9685_ADDR_BOARD_A, 1, NEUTRAL_PULSE_US, 8.0f));  // channel 1 unaffected
}

void test_same_channel_number_different_board_is_independent(void) {
    DwellGuard guard(8.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 3, 1600, 0.0f);
    TEST_ASSERT_FALSE(guard.observe(PCA9685_ADDR_BOARD_B, 3, NEUTRAL_PULSE_US, 0.0f));
    TEST_ASSERT_TRUE(guard.observe(PCA9685_ADDR_BOARD_A, 3, 1600, 8.0f));
}

void test_reset_clears_tracking(void) {
    DwellGuard guard(8.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 0.0f);
    guard.reset(PCA9685_ADDR_BOARD_A, 0);
    TEST_ASSERT_FALSE(guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 8.0f));  // would have tripped without reset
}

void test_remaining_s_counts_down(void) {
    DwellGuard guard(8.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, guard.remainingS(PCA9685_ADDR_BOARD_A, 0, 1600, 3.0f));
}

void test_remaining_s_negative_one_at_neutral(void) {
    DwellGuard guard(8.0f);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, guard.remainingS(PCA9685_ADDR_BOARD_A, 0, NEUTRAL_PULSE_US, 0.0f));
}

void test_wiggling_within_non_neutral_does_not_reset_clock(void) {
    DwellGuard guard(8.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 0.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 0, 1650, 4.0f);  // still away from neutral, different value
    TEST_ASSERT_TRUE(guard.observe(PCA9685_ADDR_BOARD_A, 0, 1620, 8.0f));
}

void test_is_away_reflects_last_observe(void) {
    DwellGuard guard(8.0f);
    TEST_ASSERT_FALSE(guard.isAway(PCA9685_ADDR_BOARD_A, 0));
    guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 0.0f);
    TEST_ASSERT_TRUE(guard.isAway(PCA9685_ADDR_BOARD_A, 0));
    guard.observe(PCA9685_ADDR_BOARD_A, 0, NEUTRAL_PULSE_US, 1.0f);
    TEST_ASSERT_FALSE(guard.isAway(PCA9685_ADDR_BOARD_A, 0));
}

void test_is_away_false_after_reset(void) {
    DwellGuard guard(8.0f);
    guard.observe(PCA9685_ADDR_BOARD_A, 0, 1600, 0.0f);
    guard.reset(PCA9685_ADDR_BOARD_A, 0);
    TEST_ASSERT_FALSE(guard.isAway(PCA9685_ADDR_BOARD_A, 0));
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_neutral_never_trips);
    RUN_TEST(test_trips_at_timeout);
    RUN_TEST(test_channels_are_independent);
    RUN_TEST(test_same_channel_number_different_board_is_independent);
    RUN_TEST(test_is_away_reflects_last_observe);
    RUN_TEST(test_is_away_false_after_reset);
    RUN_TEST(test_reset_clears_tracking);
    RUN_TEST(test_remaining_s_counts_down);
    RUN_TEST(test_remaining_s_negative_one_at_neutral);
    RUN_TEST(test_wiggling_within_non_neutral_does_not_reset_clock);
    return UNITY_END();
}
