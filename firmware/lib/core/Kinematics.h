// Pure FK/IK for one hexapod leg. C++ port of robot/kinematics.py --
// kept in lockstep via tests/fixtures/kinematics_golden.json
// (test_kinematics/ loads the generated golden_data.h, not the JSON
// directly -- see that test file for why). See reference/ANALYSIS.md
// Sections 1-3 and 7 for the full design rationale; robot/kinematics.py
// carries the same comments in full, this is the port.
//
// Degrees at the boundary, radians internally. PCA9685 pulse units never
// appear here -- that conversion is the separate AngleToPulse leaf
// function.
#pragma once

struct LegGeometry {
    const char* name;
    float originXMm;
    float originYMm;
};

// Six legs, RF/RM/RR/LR/LM/LF -- mirrors robot/kinematics.py's LEGS.
// Order matters: consecutive around the physical ring, which is what
// makes legIndex % 2 the correct tripod grouping (ANALYSIS.md Section 4).
extern const LegGeometry kLegs[6];

constexpr float kCoxaLengthMm = 38.0f;
constexpr float kFemurLengthMm = 86.25f;
constexpr float kTibiaLengthMm = 160.5f;

// Joint angle clamps -- mirrors robot/kinematics.py's bounds exactly.
// Femur/tibia are placeholders (full servo travel) until real assembled-
// frame mechanical limits are known; tibia's range is negative because
// JointAngles::tibiaDeg is raw gamma (see below), provably <= 0.
constexpr float kCoxaMinDeg = -45.0f;
constexpr float kCoxaMaxDeg = 45.0f;
constexpr float kFemurMinDeg = -90.0f;
constexpr float kFemurMaxDeg = 90.0f;
constexpr float kTibiaMinDeg = -180.0f;
constexpr float kTibiaMaxDeg = 0.0f;

float legMountAngleRad(const LegGeometry& leg);

struct Point3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Raw math convention (matches calculate_fk/calculate_ik internally) --
// NOT the servo command convention. tibiaDeg is raw gamma: provably <= 0
// for every reachable target (ANALYSIS.md Section 2's acos-range proof).
// Call toServoDeg() to get the bipolar-90/zero-based-negated convention
// the reference's Servo.write() calls actually used.
struct JointAngles {
    float coxaDeg = 0.0f;
    float femurDeg = 0.0f;
    float tibiaDeg = 0.0f;
};

// Every joint clamped, not just coxa -- closes ANALYSIS.md safety gap
// #5.1. Apply after inverseKinematics(), before a result is ever used to
// command hardware.
JointAngles clampJointAngles(const JointAngles& angles);

// Mirrors calculate_fk. originZMm is the leg's body-frame mount height
// (typically 0) -- an explicit parameter, never packed into a reused
// field (ANALYSIS.md Section 1's fix for the reference's overloaded
// Vector.z).
Point3 forwardKinematics(const JointAngles& angles, const LegGeometry& leg, float originZMm = 0.0f);

// Mirrors calculate_ik. Rotates the target into the leg's local frame by
// -mountAngleRad (a pure rotation, never a reflection -- ANALYSIS.md
// Section 3's proof that no per-leg sign flip is needed for any leg),
// clamps D into the reachable annulus before acos to avoid a domain
// error. Returns raw (unclamped) angles in the same convention
// forwardKinematics() consumes -- call clampJointAngles() and then
// toServoDeg() before commanding hardware.
JointAngles inverseKinematics(const Point3& target, const LegGeometry& leg, float originZMm = 0.0f);

struct ServoDeg {
    float coxaDeg;
    float femurDeg;
    float tibiaDeg;
};

// The one place the reference's servo-write convention is encoded:
// coxa/femur bipolar centered on 90 (matches NEUTRAL_PULSE_US); tibia
// zero-based and negated (ANALYSIS.md Section 2 -- an explicit -gamma,
// not fabs(gamma): same output, fails loudly instead of silently if the
// >= 0 invariant is ever broken by a future IK change).
ServoDeg toServoDeg(const JointAngles& angles);
