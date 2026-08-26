#include <unity.h>

#include "ArmGate.h"

void setUp(void) {}
void tearDown(void) {}

void test_starts_disarmed(void) {
    ArmGate gate(30.0f);
    TEST_ASSERT_FALSE(gate.isArmed());
}

void test_arm_then_disarm(void) {
    ArmGate gate(30.0f);
    gate.arm(0.0f);
    TEST_ASSERT_TRUE(gate.isArmed());
    gate.disarm();
    TEST_ASSERT_FALSE(gate.isArmed());
}

void test_auto_disarms_after_timeout(void) {
    ArmGate gate(30.0f);
    gate.arm(0.0f);
    gate.tick(29.9f);
    TEST_ASSERT_TRUE(gate.isArmed());
    gate.tick(30.0f);
    TEST_ASSERT_FALSE(gate.isArmed());
}

void test_refresh_extends_the_window(void) {
    ArmGate gate(30.0f);
    gate.arm(0.0f);
    gate.tick(20.0f);
    gate.refresh(20.0f);  // window now extends to 50.0
    gate.tick(45.0f);
    TEST_ASSERT_TRUE(gate.isArmed());
    gate.tick(50.0f);
    TEST_ASSERT_FALSE(gate.isArmed());
}

void test_refresh_while_disarmed_does_not_arm(void) {
    ArmGate gate(30.0f);
    gate.refresh(0.0f);
    TEST_ASSERT_FALSE(gate.isArmed());
}

void test_two_independent_gates_do_not_affect_each_other(void) {
    // Mirrors calibration_mode vs bench_mode -- arming one must never
    // arm or extend the other. See docs/protocol.md Section 8.
    ArmGate calibration(30.0f);
    ArmGate bench(60.0f);

    calibration.arm(0.0f);
    TEST_ASSERT_TRUE(calibration.isArmed());
    TEST_ASSERT_FALSE(bench.isArmed());

    bench.arm(0.0f);
    calibration.disarm();
    TEST_ASSERT_FALSE(calibration.isArmed());
    TEST_ASSERT_TRUE(bench.isArmed());
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_disarmed);
    RUN_TEST(test_arm_then_disarm);
    RUN_TEST(test_auto_disarms_after_timeout);
    RUN_TEST(test_refresh_extends_the_window);
    RUN_TEST(test_refresh_while_disarmed_does_not_arm);
    RUN_TEST(test_two_independent_gates_do_not_affect_each_other);
    return UNITY_END();
}
