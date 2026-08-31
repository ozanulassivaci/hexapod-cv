#include <unity.h>

#include <cmath>

#include "Gait.h"
#include "golden_data.h"

void setUp(void) {}
void tearDown(void) {}

// --- Golden fixture cross-check (matches robot/gait.py, see
// tests/test_gait.py and scripts/gen_gait_golden.py) --------------------

void test_golden_gait_scenarios(void) {
    for (int s = 0; s < kGoldenGaitScenarioCount; ++s) {
        const GoldenGaitScenario& scenario = kGoldenGaitScenarios[s];
        GaitState state = GaitState::initial(scenario.bodyHeight);

        int tickIdx = 0;
        int maxTick = scenario.ticks[scenario.tickCount - 1].tick;
        for (int tick = 1; tick <= maxTick; ++tick) {
            state = stepGait(state, scenario.dtS, scenario.vx, scenario.vy, scenario.speed, scenario.rotation,
                              scenario.bodyHeight);
            if (tickIdx < scenario.tickCount && scenario.ticks[tickIdx].tick == tick) {
                const GoldenGaitTick& expected = scenario.ticks[tickIdx];
                TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected.phase, state.phase);
                for (int leg = 0; leg < 6; ++leg) {
                    TEST_ASSERT_FLOAT_WITHIN(1e-2f, expected.legAngles[leg].coxaDeg, state.legAngles[leg].coxaDeg);
                    TEST_ASSERT_FLOAT_WITHIN(1e-2f, expected.legAngles[leg].femurDeg,
                                              state.legAngles[leg].femurDeg);
                    TEST_ASSERT_FLOAT_WITHIN(1e-2f, expected.legAngles[leg].tibiaDeg,
                                              state.legAngles[leg].tibiaDeg);
                }
                ++tickIdx;
            }
        }
    }
}

// --- Phase properties ----------------------------------------------------

void test_phase_wraps_into_unit_interval_over_long_run(void) {
    GaitState state = GaitState::initial();
    for (int i = 0; i < 5000; ++i) {
        state = stepGait(state, 0.02f, 1.0f, 0.0f, 80.0f, 0.0f, 50.0f);
        TEST_ASSERT_TRUE(state.phase >= 0.0f && state.phase < 1.0f);
    }
}

void test_phase_frozen_when_speed_zero(void) {
    GaitState state = GaitState::initial();
    state = stepGait(state, 0.02f, 1.0f, 0.0f, 0.0f, 0.0f, 50.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.phase);
}

void test_idle_holds_exactly_at_home_no_drift(void) {
    GaitState state = GaitState::initial(50.0f);
    for (int i = 0; i < 17; ++i) {
        state = stepGait(state, 0.02f, 1.0f, 0.0f, 80.0f, 0.0f, 50.0f);
    }
    for (int i = 0; i < 200; ++i) {
        state = stepGait(state, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 50.0f);
    }
    Point3 feet[6];
    state.footPositions(feet);
    for (int i = 0; i < 6; ++i) {
        Point3 home = homePosition(kLegs[i], 50.0f);
        TEST_ASSERT_FLOAT_WITHIN(1e-1f, home.x, feet[i].x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1f, home.y, feet[i].y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1f, home.z, feet[i].z);
    }
}

// --- Tripod grouping -------------------------------------------------------

void test_tripod_grouping_always_three_and_three(void) {
    for (int hundredths = 0; hundredths < 100; ++hundredths) {
        float phase = hundredths / 100.0f;
        int stanceCount = 0;
        for (int leg = 0; leg < 6; ++leg) {
            if (isInStance(leg, phase)) ++stanceCount;
        }
        TEST_ASSERT_EQUAL_INT(3, stanceCount);
    }
}

void test_tripod_grouping_matches_leg_index_parity(void) {
    for (int hundredths = 0; hundredths < 100; ++hundredths) {
        float phase = hundredths / 100.0f;
        bool even0 = isInStance(0, phase);
        for (int leg = 2; leg < 6; leg += 2) {
            TEST_ASSERT_TRUE(isInStance(leg, phase) == even0);
        }
        bool odd1 = isInStance(1, phase);
        for (int leg = 3; leg < 6; leg += 2) {
            TEST_ASSERT_TRUE(isInStance(leg, phase) == odd1);
        }
        TEST_ASSERT_TRUE(even0 != odd1);
    }
}

// --- Slew-rate bound -------------------------------------------------------

void test_slew_rate_bound_holds_for_worst_case_jump(void) {
    const float dtS = 0.02f;
    const float maxDelta = kMaxSlewDegPerS * dtS + 1e-4f;

    GaitState before = GaitState::initial(100.0f);
    GaitState after = stepGait(before, dtS, 1.0f, 1.0f, 100.0f, 1.0f, 0.0f);

    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_TRUE(std::fabs(after.legAngles[i].coxaDeg - before.legAngles[i].coxaDeg) <= maxDelta);
        TEST_ASSERT_TRUE(std::fabs(after.legAngles[i].femurDeg - before.legAngles[i].femurDeg) <= maxDelta);
        TEST_ASSERT_TRUE(std::fabs(after.legAngles[i].tibiaDeg - before.legAngles[i].tibiaDeg) <= maxDelta);
    }
}

// --- Body height mapping -----------------------------------------------------

void test_body_height_mapping_endpoints(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, kBodyHeightZCrouchedMm, homeZMm(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, kBodyHeightZTallMm, homeZMm(100.0f));
}

void test_body_height_mapping_monotonic(void) {
    float prev = homeZMm(0.0f);
    for (int h = 1; h <= 100; ++h) {
        float current = homeZMm(static_cast<float>(h));
        TEST_ASSERT_TRUE(current < prev);
        prev = current;
    }
}

void test_body_height_reachability_not_silently_clamped_at_extremes(void) {
    const float dMin = std::fabs(kFemurLengthMm - kTibiaLengthMm);
    const float dMax = kFemurLengthMm + kTibiaLengthMm;
    const float heights[] = {0.0f, 50.0f, 100.0f};
    for (const LegGeometry& leg : kLegs) {
        for (float height : heights) {
            Point3 target = homePosition(leg, height);
            float localX = target.x - leg.originXMm;
            float localY = target.y - leg.originYMm;
            float lXy = std::hypot(localX, localY);
            float lForward = lXy - kCoxaLengthMm;
            float d = std::hypot(lForward, target.z);
            TEST_ASSERT_TRUE(d > dMin && d < dMax);
        }
    }
}

// --- Clip stats (ANALYSIS.md Section 5.7: was silent) ----------------------
//
// The Python side (tests/test_gait.py) additionally has a test that
// forces a real clip through step()'s public path by monkeypatching
// STEP_LENGTH_MM to something absurd -- not possible here, since
// kStepLengthMm is a compile-time constexpr on purpose (a real,
// intentional guarantee, not a test-convenience gap). The overshoot
// computation itself is covered directly and precisely by
// test_kinematics/'s test_unreachable_*_target_reports_matching_*
// cases; what's left to check natively is that GaitState starts at zero
// and stays there across the verified-safe envelope, mirroring the
// Python regression guard of the same name.

void test_clip_stats_stay_zero_across_the_verified_safe_envelope(void) {
    GaitState state = GaitState::initial(50.0f);
    for (int i = 0; i < 200; ++i) {
        float vx = std::cos(i * 0.31f);
        float vy = std::sin(i * 0.17f);
        float speed = 50.0f + 50.0f * std::sin(i * 0.05f);
        float rotation = std::sin(i * 0.13f);
        float bodyHeight = 50.0f + 50.0f * std::sin(i * 0.02f);
        state = stepGait(state, 0.02f, vx, vy, speed, rotation, bodyHeight);
    }
    TEST_ASSERT_EQUAL_UINT32(0, state.ikClipCount);
    TEST_ASSERT_EQUAL_UINT32(0, state.jointClipCount);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.ikClipWorstMm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.jointClipWorstDeg);
    TEST_ASSERT_FALSE(state.clippedThisTick);
}

// --- isMotionIdle ------------------------------------------------------------

void test_is_motion_idle(void) {
    TEST_ASSERT_TRUE(isMotionIdle(0.0f, 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(isMotionIdle(1.0f, 0.0f, 0.0f, 0.0f));  // speed 0 -- idle regardless of vx/vy
    TEST_ASSERT_TRUE(isMotionIdle(0.0f, 0.0f, 50.0f, 0.0f));  // no direction/rotation -- idle
    TEST_ASSERT_FALSE(isMotionIdle(1.0f, 0.0f, 50.0f, 0.0f));
    TEST_ASSERT_FALSE(isMotionIdle(0.0f, 0.0f, 50.0f, 0.5f));  // rotation alone -- not idle
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_golden_gait_scenarios);
    RUN_TEST(test_phase_wraps_into_unit_interval_over_long_run);
    RUN_TEST(test_phase_frozen_when_speed_zero);
    RUN_TEST(test_idle_holds_exactly_at_home_no_drift);
    RUN_TEST(test_tripod_grouping_always_three_and_three);
    RUN_TEST(test_tripod_grouping_matches_leg_index_parity);
    RUN_TEST(test_slew_rate_bound_holds_for_worst_case_jump);
    RUN_TEST(test_body_height_mapping_endpoints);
    RUN_TEST(test_body_height_mapping_monotonic);
    RUN_TEST(test_body_height_reachability_not_silently_clamped_at_extremes);
    RUN_TEST(test_clip_stats_stay_zero_across_the_verified_safe_envelope);
    RUN_TEST(test_is_motion_idle);
    return UNITY_END();
}
