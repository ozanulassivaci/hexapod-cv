// Pure tripod gait engine: phase advance, per-leg foot-target
// generation, slew-rate-limited joint tracking. C++ port of
// robot/gait.py -- kept in lockstep via tests/fixtures/gait_golden.json
// (test_gait/ loads the generated golden_data.h, same reasoning as
// test_kinematics/ -- see that header). robot/gait.py carries the full
// design rationale in comments; this is the port.
#pragma once

#include "Kinematics.h"

constexpr float kStepLengthMm = 60.0f;

// STEP_HEIGHT=100 was ported directly from the reference (a real,
// measured bug there, not a porting error -- traced exactly: step_z =
// sin(phase_norm*pi)*STEP_HEIGHT, no other scaling anywhere in the
// chain; the reference's own position smoothing barely attenuates it,
// and this port's slew-rate limiting attenuates it not at all, so
// whatever this constant is IS the real achieved peak). At 100, real
// peak foot lift puts the foot above the coxa mounting plane at every
// sane standing height. 20mm sits inside "typical hexapod swing height
// (20-40mm)" with ~10mm of clearance below the coxa plane even at the
// shallowest configured stance during a full-speed walk -- see
// robot/gait.py's STEP_HEIGHT_MM for the full derivation.
constexpr float kStepHeightMm = 20.0f;

constexpr float kRotationStepRad = 15.0f * 0.017453292519943295f;  // 15 deg in radians

// Cycles/second at speed=100 -- matches the reference's t += 0.015 per
// 10ms tick (1.5 Hz), the only cadence Hexapod_Arduino.ino was ever
// validated at.
constexpr float kBasePhaseRateHz = 1.5f;

// Body height (0..100) linearly maps onto this home.z range. Chosen by
// sweeping D (leg extension) and torque-sensitivity across the full
// theoretical range for this leg's home footprint -- D_min (fully
// folded) is unreachable at any height here, D_max (fully extended, a
// kinematic singularity) is the real constraint the old range
// (-20/-150) got too close to (93% of max reach standing still, and gait
// motion pushed it over entirely in a full envelope sweep). This range
// keeps the worst case at 5.4% margin from D_max with zero clipping
// across the same sweep, and is centered near the best-conditioned point
// (tibia raw angle ~ -90 deg). Costs 40mm of max standing height and
// 10mm of min crouch; walking dynamics are untouched. See
// robot/gait.py's BODY_HEIGHT_Z_CROUCHED_MM/TALL_MM for the full
// derivation.
constexpr float kBodyHeightZCrouchedMm = -30.0f;  // height=0
constexpr float kBodyHeightZTallMm = -110.0f;  // height=100

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
