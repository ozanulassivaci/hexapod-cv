// Pure tripod gait engine: phase advance, per-leg foot-target
// generation, slew-rate-limited joint tracking. C++ port of
// robot/gait.py -- kept in lockstep via tests/fixtures/gait_golden.json
// (test_gait/ loads the generated golden_data.h, same reasoning as
// test_kinematics/ -- see that header). robot/gait.py carries the full
// design rationale in comments; this is the port.
#pragma once

#include "Kinematics.h"

constexpr float kStepLengthMm = 60.0f;
constexpr float kStepHeightMm = 100.0f;
constexpr float kRotationStepRad = 15.0f * 0.017453292519943295f;  // 15 deg in radians

// Cycles/second at speed=100 -- matches the reference's t += 0.015 per
// 10ms tick (1.5 Hz), the only cadence Hexapod_Arduino.ino was ever
// validated at.
constexpr float kBasePhaseRateHz = 1.5f;

// Body height (0..100) linearly maps onto this home.z range. Placeholder
// bracketing the reference's compile-time DEFAULT_Z=-40mm -- narrow once
// real reachability/ground-clearance data exists (see
// tests/test_gait.py::test_body_height_reachability_not_silently_clamped_at_extremes,
// mirrored in this module's own native test).
constexpr float kBodyHeightZCrouchedMm = -20.0f;  // height=0
constexpr float kBodyHeightZTallMm = -150.0f;  // height=100

// Per-joint slew-rate bound, degrees/second -- see robot/gait.py's
// MAX_SLEW_DEG_PER_S for the full rationale (ANALYSIS.md safety gap #3).
constexpr float kMaxSlewDegPerS = 300.0f;

constexpr float kDefaultBodyHeight = 50.0f;

// coxaDeg=0, femurDeg=45, tibiaDeg=-90 -- matches the reference's
// neutralAngles(0, pi/4, -pi/2) exactly.
extern const JointAngles kNeutralStanceAngles;

float homeZMm(float bodyHeight);
Point3 homePosition(const LegGeometry& leg, float bodyHeight);

// True if this leg is in its foot-planted half of the tripod cycle at
// the given global phase -- ANALYSIS.md Section 4's confirmed-correct
// legIndex % 2 grouping.
bool isInStance(int legIndex, float phase);

// The same idle condition footTarget() uses internally, exposed so
// main.cpp's bench-mode arm-refusal check ("refuse to arm bench_mode
// while gait is actively non-idle") can reuse it instead of re-deriving
// it -- one definition of "idle".
bool isMotionIdle(float vx, float vy, float speed, float rotation);

// Mirrors get_foot_target, generalized -- see robot/gait.py::foot_target
// for the full rationale (continuous speed scaling, strafing,
// magnitude-jointly-clamped vx/vy).
Point3 footTarget(int legIndex, const LegGeometry& leg, float phase, float vx, float vy, float speed,
                   float rotation, float bodyHeight);

struct GaitState {
    float phase = 0.0f;
    float bodyHeight = kDefaultBodyHeight;
    JointAngles legAngles[6];

    static GaitState initial(float bodyHeight = kDefaultBodyHeight);

    // FK of every leg's current tracked angles -- for telemetry/
    // visualization, mirrors robot/gait.py's GaitState.foot_positions().
    void footPositions(Point3 outPositions[6]) const;
};

// Advances phase (frozen when speed ~= 0) and slew-limits every joint
// toward its freshly-computed IK goal by at most kMaxSlewDegPerS * dtS --
// see robot/gait.py::step for the full rationale. dtS must be real
// elapsed time; this has no clock of its own.
GaitState stepGait(const GaitState& state, float dtS, float vx, float vy, float speed, float rotation,
                    float bodyHeight);

// Thin stateful wrapper matching this firmware's other lib/core modules'
// style (ArmGate, DwellGuard, LinkWatchdog). Deliberately has no
// freeze()/unfreeze() of its own -- the caller (main.cpp) decides
// whether to call tick() at all on a given cycle (skipping it is what
// "gait stops advancing and stops emitting pulses" means in practice,
// both for SafetyMode::Assembled's fault path and for bench-mode mutual
// exclusion), which is simpler and can't desync the way a persisted
// frozen flag that something forgets to clear could.
class GaitEngine {
public:
    explicit GaitEngine(float bodyHeight = kDefaultBodyHeight) : state_(GaitState::initial(bodyHeight)) {}

    void tick(float dtS, float vx, float vy, float speed, float rotation, float bodyHeight) {
        state_ = stepGait(state_, dtS, vx, vy, speed, rotation, bodyHeight);
    }

    const GaitState& state() const { return state_; }

private:
    GaitState state_;
};
