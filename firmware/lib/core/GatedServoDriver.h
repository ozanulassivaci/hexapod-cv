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

#include "ServoOutput.h"
#include "ServoProfileStore.h"

class GatedServoDriver {
public:
    GatedServoDriver(ServoOutput& output, ServoProfileStore& profiles);

    // Returns true if applied. Returns false (and does not call the
    // underlying ServoOutput at all) if hasServoIndex is true and
    // degFromNeutral falls outside that servo's recorded min/max --
    // reject, not clamp, so the operator gets a clear ack/log line rather
    // than a silently truncated value with no feedback (see the design
    // discussion this was built from). reason is set to a static string
    // on refusal. degFromNeutral is the caller's job to compute (see
    // ServoMap.h's neutralPulseUsFor/AngleToPulse.h's
    // pulseUsToDegFromNeutral, or driveGaitOutputs' own servoDeg minus
    // neutralServoDeg) -- ignored when hasServoIndex is false, since
    // there's no profile to compare it against. Enforcement compares
    // degrees, not the pulse actually being written, so the same
    // bench-recorded limit applies correctly regardless of that specific
    // servo's offsetUs (see ServoProfile.h).
    bool commandPulse(uint8_t board, uint8_t channel, uint16_t pulseUs, bool hasServoIndex,
                       uint8_t servoIndex, float degFromNeutral, const char** reason);

    void release(uint8_t board, uint8_t channel);
    void releaseAll();

private:
    ServoOutput& output_;
    ServoProfileStore& profiles_;
};
