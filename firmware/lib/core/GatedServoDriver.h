// The only way any code in this firmware may command a raw pulse. This is
// what makes limit enforcement "impossible to bypass from any code path":
// ServoOutput::setPulse is never called directly outside this class, and
// this class refuses (never forwards to ServoOutput at all) a value that
// falls outside the recorded limits for the given servo_index, if known.
//
// No servo_index (a channel not yet assigned a future position) means
// there's nothing to look up -- only the generic protocol-level bound
// applies, already enforced by decodeCommand before a value ever reaches
// here. See docs/protocol.md Section 8 for why bench_pulse's servo_index
// is optional in the first place.
#pragma once

#include <cstdint>

#include "Config.h"
#include "ServoOutput.h"
#include "ServoProfileStore.h"

// Which of the two currently-conflated-until-now limit tiers to enforce
// against a servo's recorded min/maxDegFromNeutral (the *mechanical*
// tier -- see ServoProfile.h -- where printed parts actually collide):
//
// - Mechanical: compare against the recorded bound exactly as measured.
//   Bench mode's own tier -- exploration (Test Leg work, bench_pulse)
//   must be able to reach and probe past the true mechanical edge to
//   find it; a limit that stopped short by a safety margin would make
//   the margin itself unmeasurable.
// - Safe: compare against the recorded bound shrunk inward by
//   SAFE_LIMIT_MARGIN_DEG on each end (Config.h). Gait's tier, always --
//   driveGaitOutputs is the only caller that ever passes this, and it is
//   architecturally the only path gait-computed pulses can reach
//   hardware through, so this bound is unreachable by any gait-driven
//   code path, not just unreachable by convention.
//
// The gait envelope (the third tier: what gait's own math ever actually
// asks for, informational only, never itself a constraint -- see
// robot/gait.py's GAIT_ENVELOPE_* constants) never appears here at all;
// it's a design-time fact checked offline (robot/safe_limit_check.py)
// against whatever a Test Leg session has recorded, not a runtime
// enforcement path.
enum class LimitMode : uint8_t { Mechanical, Safe };

class GatedServoDriver {
public:
    GatedServoDriver(ServoOutput& output, ServoProfileStore& profiles);

    // Returns true if applied. Returns false (and does not call the
    // underlying ServoOutput at all) if hasServoIndex is true and
    // degFromNeutral falls outside that servo's recorded min/max (after
    // LimitMode's margin, if any) -- reject, not clamp, so the operator
    // gets a clear ack/log line rather than a silently truncated value
    // with no feedback (see the design discussion this was built from).
    // reason is set to a static string on refusal. degFromNeutral is the
    // caller's job to compute (see ServoMap.h's neutralPulseUsFor/
    // AngleToPulse.h's pulseUsToDegFromNeutral, or driveGaitOutputs' own
    // servoDeg minus neutralServoDeg) -- ignored when hasServoIndex is
    // false, since there's no profile to compare it against. Enforcement
    // compares degrees, not the pulse actually being written, so the
    // same bench-recorded limit applies correctly regardless of that
    // specific servo's offsetUs (see ServoProfile.h).
    bool commandPulse(uint8_t board, uint8_t channel, uint16_t pulseUs, bool hasServoIndex,
                       uint8_t servoIndex, float degFromNeutral, LimitMode limitMode, const char** reason);

    void release(uint8_t board, uint8_t channel);
    void releaseAll();

private:
    ServoOutput& output_;
    ServoProfileStore& profiles_;
};
