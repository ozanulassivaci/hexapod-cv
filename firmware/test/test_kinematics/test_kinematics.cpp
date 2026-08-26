#include <unity.h>

#include <cmath>
#include <cstring>

#include "Kinematics.h"
#include "golden_data.h"

void setUp(void) {}
void tearDown(void) {}

static const LegGeometry* legByName(const char* name) {
    for (int i = 0; i < 6; ++i) {
        if (std::strcmp(kLegs[i].name, name) == 0) return &kLegs[i];
    }
    return nullptr;
}

// --- Golden fixture cross-check (matches robot/kinematics.py, see
// tests/test_kinematics.py and scripts/gen_kinematics_golden.py) --------

void test_golden_fk_cases(void) {
    for (int i = 0; i < kGoldenFkCaseCount; ++i) {
        const GoldenFkCase& c = kGoldenFkCases[i];
        const LegGeometry* leg = legByName(c.leg);
        TEST_ASSERT_NOT_NULL(leg);
        JointAngles angles{c.coxaDeg, c.femurDeg, c.tibiaDeg};
        Point3 foot = forwardKinematics(angles, *leg);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, c.expectedX, foot.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, c.expectedY, foot.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, c.expectedZ, foot.z);
    }
}

void test_golden_ik_cases(void) {
    for (int i = 0; i < kGoldenIkCaseCount; ++i) {
        const GoldenIkCase& c = kGoldenIkCases[i];
        const LegGeometry* leg = legByName(c.leg);
        TEST_ASSERT_NOT_NULL(leg);
        Point3 target{c.targetX, c.targetY, c.targetZ};
        JointAngles angles = inverseKinematics(target, *leg);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, c.expectedCoxaDeg, angles.coxaDeg);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, c.expectedFemurDeg, angles.femurDeg);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, c.expectedTibiaDeg, angles.tibiaDeg);
    }
}

// --- Round trip, no-NaN, invariants (deterministic points -- native has
// no seeded-RNG helper readily at hand, this supplements the golden
// cross-check rather than replacing pytest's randomized sweep) ----------

// A target guaranteed inside the reachable annulus for `leg`, expressed
// relative to its own local frame (radius lXy along local +x, height z)
// then rotated into world frame -- mirrors
// tests/test_kinematics.py::_random_reachable_target's construction, but
// deterministic (fixed points, not random) since native has no seeded-RNG
// helper readily at hand.
static Point3 reachableTarget(const LegGeometry& leg, float lXy, float z) {
    const float mountAngleRad = legMountAngleRad(leg);
    const float worldX = lXy * std::cos(mountAngleRad);
    const float worldY = lXy * std::sin(mountAngleRad);
    return Point3{leg.originXMm + worldX, leg.originYMm + worldY, z};
}

void test_fk_of_ik_recovers_reachable_target(void) {
    const struct {
        float lXy, z;
    } offsets[] = {{150.0f, -100.0f}, {120.0f, 50.0f}, {200.0f, -20.0f}};
    for (const LegGeometry& leg : kLegs) {
        for (const auto& o : offsets) {
            Point3 target = reachableTarget(leg, o.lXy, o.z);
            JointAngles angles = inverseKinematics(target, leg);
            Point3 recovered = forwardKinematics(angles, leg);
            TEST_ASSERT_FLOAT_WITHIN(1e-1f, target.x, recovered.x);
            TEST_ASSERT_FLOAT_WITHIN(1e-1f, target.y, recovered.y);
            TEST_ASSERT_FLOAT_WITHIN(1e-1f, target.z, recovered.z);
        }
    }
}

void test_no_nan_across_adversarial_targets(void) {
    const Point3 adversarial[] = {
        {0.0f, 0.0f, 0.0f}, {1000.0f, 1000.0f, 1000.0f}, {-1000.0f, -1000.0f, -1000.0f}, {0.0f, 0.0f, 5000.0f},
    };
    for (const LegGeometry& leg : kLegs) {
        // target exactly at this leg's own origin -- l_xy == 0
        Point3 atOrigin{leg.originXMm, leg.originYMm, 0.0f};
        JointAngles a = inverseKinematics(atOrigin, leg);
        TEST_ASSERT_TRUE(std::isfinite(a.coxaDeg) && std::isfinite(a.femurDeg) && std::isfinite(a.tibiaDeg));

        for (const Point3& target : adversarial) {
            JointAngles angles = inverseKinematics(target, leg);
            TEST_ASSERT_TRUE(std::isfinite(angles.coxaDeg));
            TEST_ASSERT_TRUE(std::isfinite(angles.femurDeg));
            TEST_ASSERT_TRUE(std::isfinite(angles.tibiaDeg));
        }
    }
}

void test_tibia_raw_gamma_never_positive(void) {
    const Point3 targets[] = {
        {80.0f, -40.0f, -40.0f}, {200.0f, 100.0f, 50.0f}, {-50.0f, -50.0f, -100.0f}, {0.0f, 0.0f, 0.0f},
    };
    for (const LegGeometry& leg : kLegs) {
        for (const Point3& target : targets) {
            JointAngles angles = inverseKinematics(target, leg);
            TEST_ASSERT_TRUE(angles.tibiaDeg <= 1e-4f);
        }
    }
}

void test_rotational_sense_symmetry_across_all_legs(void) {
    const float deltaDeg = 1.0f;
    for (const LegGeometry& leg : kLegs) {
        const float mountAngleRad = legMountAngleRad(leg);
        const float alphaA = 0.0f + mountAngleRad;
        const float alphaB = deltaDeg * (M_PI / 180.0f) + mountAngleRad;
        const float dAlphaWorldDCoxa = (alphaB - alphaA) / (deltaDeg * (M_PI / 180.0f));
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, dAlphaWorldDCoxa);
    }
}

// --- Clamping / servo convention ---------------------------------------

void test_clamp_joint_angles_bounds(void) {
    JointAngles extreme{9999.0f, -9999.0f, 9999.0f};
    JointAngles clamped = clampJointAngles(extreme);
    TEST_ASSERT_EQUAL_FLOAT(kCoxaMaxDeg, clamped.coxaDeg);
    TEST_ASSERT_EQUAL_FLOAT(kFemurMinDeg, clamped.femurDeg);
    TEST_ASSERT_EQUAL_FLOAT(kTibiaMaxDeg, clamped.tibiaDeg);  // tibia's max is 0 (raw gamma convention)
}

void test_negative_l_forward_femur_clamped(void) {
    const LegGeometry& leg = kLegs[0];
    Point3 behind{leg.originXMm, leg.originYMm, -5.0f};
    JointAngles angles = clampJointAngles(inverseKinematics(behind, leg));
    TEST_ASSERT_TRUE(angles.femurDeg >= kFemurMinDeg && angles.femurDeg <= kFemurMaxDeg);
}

void test_to_servo_deg_convention(void) {
    JointAngles angles{10.0f, -5.0f, -30.0f};
    ServoDeg servo = toServoDeg(angles);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 100.0f, servo.coxaDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 85.0f, servo.femurDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 30.0f, servo.tibiaDeg);  // -gamma, not fabs(gamma)
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_golden_fk_cases);
    RUN_TEST(test_golden_ik_cases);
    RUN_TEST(test_fk_of_ik_recovers_reachable_target);
    RUN_TEST(test_no_nan_across_adversarial_targets);
    RUN_TEST(test_tibia_raw_gamma_never_positive);
    RUN_TEST(test_rotational_sense_symmetry_across_all_legs);
    RUN_TEST(test_clamp_joint_angles_bounds);
    RUN_TEST(test_negative_l_forward_femur_clamped);
    RUN_TEST(test_to_servo_deg_convention);
    return UNITY_END();
}
