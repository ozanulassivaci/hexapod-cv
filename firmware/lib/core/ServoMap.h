// Flat servo_index -> hardware address table. Servo hardware addressing
// is channel-scoped, not leg-scoped (CLAUDE.md's architecture rule; see
// also reference/ANALYSIS.md Section 7's "per-leg struct vs. flat
// SERVO_MAP" split) -- Kinematics.h's kLegs holds kinematic identity
// only (mount origin, segment lengths), this table is the only place
// board/channel wiring facts live.
//
// PLACEHOLDER -- systematic default wiring (leg 0-2 on board A, channels
// 0-8; leg 3-5 on board B, channels 0-8; 3 consecutive channels per leg
// in coxa/femur/tibia order), not verified against any physical wiring.
// FILL IN / VERIFY AFTER ASSEMBLY, same caveat as Config.h's
// PCA9685_OSC_FREQ placeholders -- nothing is assembled yet (CLAUDE.md).
#pragma once

#include <cstdint>

#include "Config.h"

enum class JointType : uint8_t { Coxa = 0, Femur = 1, Tibia = 2 };

struct ServoMapEntry {
    uint8_t legIndex;  // index into Kinematics.h's kLegs
    JointType joint;
    uint8_t board;
    uint8_t channel;
};

// Indexed by servo_index (0-17) = legIndex * 3 + joint, matching this
// file's placeholder wiring exactly -- see ServoMap.cpp.
extern const ServoMapEntry kServoMap[SERVO_COUNT];

// servoIndexFor(2, JointType::Femur) == 7, etc. -- the inverse of the
// placeholder layout above, used by main.cpp to look up which
// (board, channel, ServoProfile) a given leg/joint's computed pulse goes
// to and whose calibration it's commanded through.
constexpr uint8_t servoIndexFor(uint8_t legIndex, JointType joint) {
    return legIndex * 3 + static_cast<uint8_t>(joint);
}
