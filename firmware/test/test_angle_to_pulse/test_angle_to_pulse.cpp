#include <unity.h>

#include "AngleToPulse.h"
#include "Config.h"

void setUp(void) {}
void tearDown(void) {}

void test_coxa_femur_neutral_gives_neutral_pulse(void) {
    uint16_t pulse = angleToPulseUs(90.0f, 90.0f, NEUTRAL_PULSE_US, 1, 0);
    TEST_ASSERT_EQUAL_UINT16(NEUTRAL_PULSE_US, pulse);
}

void test_tibia_neutral_gives_tibia_neutral_pulse(void) {
    uint16_t pulse = angleToPulseUs(0.0f, 0.0f, TIBIA_NEUTRAL_PULSE_US, 1, 0);
    TEST_ASSERT_EQUAL_UINT16(TIBIA_NEUTRAL_PULSE_US, pulse);
}

void test_increasing_servo_deg_increases_pulse_with_positive_sign(void) {
    uint16_t low = angleToPulseUs(60.0f, 90.0f, NEUTRAL_PULSE_US, 1, 0);
    uint16_t mid = angleToPulseUs(90.0f, 90.0f, NEUTRAL_PULSE_US, 1, 0);
    uint16_t high = angleToPulseUs(120.0f, 90.0f, NEUTRAL_PULSE_US, 1, 0);
    TEST_ASSERT_TRUE(low < mid);
    TEST_ASSERT_TRUE(mid < high);
}

void test_negative_sign_flips_direction_around_this_joints_own_neutral(void) {
    // Sign must flip around neutralPulseUs (1500 for coxa/femur, NOT
    // tibia's different neutral), or the tibia joint's zero-based
    // convention would flip around the wrong point entirely.
    uint16_t plus = angleToPulseUs(120.0f, 90.0f, NEUTRAL_PULSE_US, 1, 0);
    uint16_t minus = angleToPulseUs(120.0f, 90.0f, NEUTRAL_PULSE_US, -1, 0);
    int16_t deltaPlus = static_cast<int16_t>(plus) - NEUTRAL_PULSE_US;
    int16_t deltaMinus = static_cast<int16_t>(minus) - NEUTRAL_PULSE_US;
    TEST_ASSERT_EQUAL_INT16(-deltaPlus, deltaMinus);
}

void test_offset_shifts_pulse_linearly(void) {
    uint16_t base = angleToPulseUs(90.0f, 90.0f, NEUTRAL_PULSE_US, 1, 0);
    uint16_t withOffset = angleToPulseUs(90.0f, 90.0f, NEUTRAL_PULSE_US, 1, 30);
    TEST_ASSERT_EQUAL_UINT16(base + 30, withOffset);
}

void test_extreme_servo_deg_clamps_to_envelope(void) {
    uint16_t high = angleToPulseUs(9999.0f, 90.0f, NEUTRAL_PULSE_US, 1, 0);
    uint16_t low = angleToPulseUs(-9999.0f, 90.0f, NEUTRAL_PULSE_US, 1, 0);
    TEST_ASSERT_EQUAL_UINT16(BENCH_PULSE_MAX_US, high);
    TEST_ASSERT_EQUAL_UINT16(BENCH_PULSE_MIN_US, low);
}

void test_extreme_offset_clamps_to_envelope(void) {
    uint16_t pulse = angleToPulseUs(90.0f, 90.0f, NEUTRAL_PULSE_US, 1, 30000);
    TEST_ASSERT_EQUAL_UINT16(BENCH_PULSE_MAX_US, pulse);
}

// --- pulseUsToDegFromNeutral: the inverse relationship ---------------------

void test_pulse_at_neutral_gives_zero_degrees(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pulseUsToDegFromNeutral(NEUTRAL_PULSE_US, NEUTRAL_PULSE_US));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pulseUsToDegFromNeutral(TIBIA_NEUTRAL_PULSE_US, TIBIA_NEUTRAL_PULSE_US));
}

void test_pulse_to_deg_matches_bench_envelope_endpoints(void) {
    // Coxa/femur: [500, 2500]us around neutral 1500 spans exactly [-90, +90]
    // -- NEUTRAL_PULSE_US is a fixed hobby-servo fact, not mount-dependent.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -90.0f, pulseUsToDegFromNeutral(BENCH_PULSE_MIN_US, NEUTRAL_PULSE_US));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f, pulseUsToDegFromNeutral(BENCH_PULSE_MAX_US, NEUTRAL_PULSE_US));
    // Tibia: zero-based, but TIBIA_NEUTRAL_PULSE_US is a per-mount
    // calibration value (docs/HOW_TO_USE.md's mounting section), not
    // fixed at either bench envelope endpoint -- computed from the live
    // constant rather than hardcoded, so this doesn't silently start
    // failing the next time that constant is updated from a real
    // measurement.
    float usPerDeg = (static_cast<float>(BENCH_PULSE_MAX_US) - static_cast<float>(BENCH_PULSE_MIN_US)) / 180.0f;
    float expectedLow = (static_cast<float>(BENCH_PULSE_MIN_US) - static_cast<float>(TIBIA_NEUTRAL_PULSE_US)) / usPerDeg;
    float expectedHigh = (static_cast<float>(BENCH_PULSE_MAX_US) - static_cast<float>(TIBIA_NEUTRAL_PULSE_US)) / usPerDeg;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expectedLow, pulseUsToDegFromNeutral(BENCH_PULSE_MIN_US, TIBIA_NEUTRAL_PULSE_US));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expectedHigh, pulseUsToDegFromNeutral(BENCH_PULSE_MAX_US, TIBIA_NEUTRAL_PULSE_US));
}

void test_pulse_to_deg_is_the_inverse_of_angle_to_pulse_at_zero_offset(void) {
    // With sign=1, offsetUs=0 (bench mode's own convention -- no
    // calibration applied), angleToPulseUs and pulseUsToDegFromNeutral
    // must be exact inverses of each other around any neutral.
    for (float deg = -80.0f; deg <= 80.0f; deg += 10.0f) {
        uint16_t pulse = angleToPulseUs(90.0f + deg, 90.0f, NEUTRAL_PULSE_US, 1, 0);
        float recovered = pulseUsToDegFromNeutral(pulse, NEUTRAL_PULSE_US);
        TEST_ASSERT_FLOAT_WITHIN(1e-1f, deg, recovered);
    }
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_coxa_femur_neutral_gives_neutral_pulse);
    RUN_TEST(test_tibia_neutral_gives_tibia_neutral_pulse);
    RUN_TEST(test_increasing_servo_deg_increases_pulse_with_positive_sign);
    RUN_TEST(test_negative_sign_flips_direction_around_this_joints_own_neutral);
    RUN_TEST(test_offset_shifts_pulse_linearly);
    RUN_TEST(test_extreme_servo_deg_clamps_to_envelope);
    RUN_TEST(test_extreme_offset_clamps_to_envelope);
    RUN_TEST(test_pulse_at_neutral_gives_zero_degrees);
    RUN_TEST(test_pulse_to_deg_matches_bench_envelope_endpoints);
    RUN_TEST(test_pulse_to_deg_is_the_inverse_of_angle_to_pulse_at_zero_offset);
    return UNITY_END();
}
