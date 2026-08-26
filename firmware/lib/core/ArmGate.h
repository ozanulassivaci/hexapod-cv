// Generic arm/disarm-with-auto-timeout gate, reused for both
// calibration_mode and bench_mode -- they have identical semantics (armed
// boolean + auto-disarm timeout + refresh-on-activity), just different
// timeout constants and independent instances (arming one must never arm
// or extend the other -- see docs/protocol.md Section 8). One
// implementation, two instances, rather than two copies that could drift
// apart from each other.
#pragma once

class ArmGate {
public:
    explicit ArmGate(float timeoutS);

    void arm(float nowS);
    void disarm();

    // Call on any gated activity while armed, to refresh the window --
    // "not a fixed wall-clock window regardless of activity", per
    // docs/protocol.md.
    void refresh(float nowS);

    // Call every control-loop tick, before checking isArmed(). Auto-disarms
    // if the window has elapsed.
    void tick(float nowS);

    bool isArmed() const { return armed_; }

private:
    float timeoutS_;
    bool armed_ = false;
    float armedUntilS_ = 0.0f;
};
