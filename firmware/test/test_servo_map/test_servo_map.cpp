#include <unity.h>

#include "Config.h"
#include "ServoMap.h"

void setUp(void) {}
void tearDown(void) {}

void test_every_index_maps_to_a_valid_board_and_channel(void) {
    for (int i = 0; i < SERVO_COUNT; ++i) {
        const ServoMapEntry& e = kServoMap[i];
        TEST_ASSERT_TRUE(e.board == PCA9685_ADDR_BOARD_A || e.board == PCA9685_ADDR_BOARD_B);
        TEST_ASSERT_TRUE(e.channel < PCA9685_CHANNELS_PER_BOARD);
        TEST_ASSERT_TRUE(e.legIndex < 6);
    }
}

void test_no_two_servos_share_a_physical_channel(void) {
    for (int i = 0; i < SERVO_COUNT; ++i) {
        for (int j = i + 1; j < SERVO_COUNT; ++j) {
            bool sameChannel = kServoMap[i].board == kServoMap[j].board && kServoMap[i].channel == kServoMap[j].channel;
            TEST_ASSERT_FALSE(sameChannel);
        }
    }
}

void test_every_leg_has_exactly_one_coxa_femur_tibia(void) {
    for (uint8_t leg = 0; leg < 6; ++leg) {
        bool sawCoxa = false, sawFemur = false, sawTibia = false;
        for (int i = 0; i < SERVO_COUNT; ++i) {
            if (kServoMap[i].legIndex != leg) continue;
            if (kServoMap[i].joint == JointType::Coxa) sawCoxa = true;
            if (kServoMap[i].joint == JointType::Femur) sawFemur = true;
            if (kServoMap[i].joint == JointType::Tibia) sawTibia = true;
        }
        TEST_ASSERT_TRUE(sawCoxa && sawFemur && sawTibia);
    }
}

void test_servo_index_for_matches_table_layout(void) {
    for (uint8_t leg = 0; leg < 6; ++leg) {
        TEST_ASSERT_EQUAL_UINT8(leg, kServoMap[servoIndexFor(leg, JointType::Coxa)].legIndex);
        TEST_ASSERT_TRUE(kServoMap[servoIndexFor(leg, JointType::Coxa)].joint == JointType::Coxa);
        TEST_ASSERT_TRUE(kServoMap[servoIndexFor(leg, JointType::Femur)].joint == JointType::Femur);
        TEST_ASSERT_TRUE(kServoMap[servoIndexFor(leg, JointType::Tibia)].joint == JointType::Tibia);
    }
}

void test_neutral_convention_matches_kinematics_servo_write(void) {
    // Coxa/femur bipolar around 90/NEUTRAL_PULSE_US, tibia zero-based
    // around 0/TIBIA_NEUTRAL_PULSE_US -- Kinematics.h's toServoDeg().
    TEST_ASSERT_EQUAL_FLOAT(90.0f, neutralServoDegFor(JointType::Coxa));
    TEST_ASSERT_EQUAL_FLOAT(90.0f, neutralServoDegFor(JointType::Femur));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, neutralServoDegFor(JointType::Tibia));
    TEST_ASSERT_EQUAL_UINT16(NEUTRAL_PULSE_US, neutralPulseUsFor(JointType::Coxa));
    TEST_ASSERT_EQUAL_UINT16(NEUTRAL_PULSE_US, neutralPulseUsFor(JointType::Femur));
    TEST_ASSERT_EQUAL_UINT16(TIBIA_NEUTRAL_PULSE_US, neutralPulseUsFor(JointType::Tibia));
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_every_index_maps_to_a_valid_board_and_channel);
    RUN_TEST(test_no_two_servos_share_a_physical_channel);
    RUN_TEST(test_every_leg_has_exactly_one_coxa_femur_tibia);
    RUN_TEST(test_servo_index_for_matches_table_layout);
    RUN_TEST(test_neutral_convention_matches_kinematics_servo_write);
    return UNITY_END();
}
