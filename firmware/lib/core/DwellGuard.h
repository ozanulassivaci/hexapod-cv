// Firmware-side stall protection for bench_pulse. Deliberately independent
// of the PC-side GUI's control/bench.py DwellGuard, and not just a port of
// it -- "don't rely on the GUI to enforce dwell... the firmware must
// protect the servo even if the GUI crashes or the link drops."
//
// Tracks all 32 physical (board, channel) addresses independently, not
// just "whichever one is currently selected" -- if the GUI's dwell guard
// only tracks its own single currently-selected channel, switching which
// channel is selected abandons the previous one's actual physical state
// with no one watching it anymore. That's a real gap in the GUI-side
// implementation this firmware-side design is built to not repeat.
//
// Same rule as the GUI's version: fires on ANY sustained non-neutral hold,
// not only a hold at a recorded limit -- the most dangerous moment is
// before a limit is known. An active sweep should not be run through this
// (see docs/protocol.md's bench design note) -- it's continuously moving,
// not stalling.
#pragma once

#include <cstdint>

#include "Config.h"

class DwellGuard {
public:
    explicit DwellGuard(float timeoutS);

    // Call every control-loop tick for every channel currently holding a
    // commanded (non-neutral) pulse. Returns true if this channel should
    // be force-released now.
    bool observe(uint8_t board, uint8_t channel, uint16_t pulseUs, float nowS);

    // Seconds until auto-release for this channel, or -1 if not currently
    // at risk (at neutral, or never observed away from neutral).
    float remainingS(uint8_t board, uint8_t channel, uint16_t pulseUs, float nowS) const;

    // Call after a channel has been released (by this guard firing, by an
    // explicit park, by safe state, etc.) so its clock doesn't keep
    // ticking against stale state.
    void reset(uint8_t board, uint8_t channel);

    // Whether this channel is currently tracked as away from neutral, as
    // of the last observe() call -- used by the OTA gate (docs/protocol.md
    // Section 8: OTA refused while any servo is commanded away from
    // neutral) without needing a second, separately-maintained tracking
    // array.
    bool isAway(uint8_t board, uint8_t channel) const;

private:
    static int indexFor(uint8_t board, uint8_t channel);

    float timeoutS_;
    bool awaySet_[2 * PCA9685_CHANNELS_PER_BOARD] = {false};
    float awaySinceS_[2 * PCA9685_CHANNELS_PER_BOARD] = {0.0f};
};
