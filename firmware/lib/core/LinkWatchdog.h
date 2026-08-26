// Failsafe contract from docs/protocol.md: track time since the last
// packet that passed sequence-freshness; if that exceeds the configured
// timeout, the link is considered down. Pure -- the caller supplies "now"
// explicitly, this never reads a clock itself, so it's unit-testable on
// the host with fabricated timestamps and no real waiting.
//
// Before any packet has ever arrived, hasTimedOut() returns true. This is
// deliberate, not an edge case to special-case away: it makes "just
// booted, nothing heard yet" and "watchdog tripped" the same state,
// matching the boot-time decision in SafeState.h -- command nothing at
// boot, because boot state and failsafe state should be identical for the
// same underlying reason (no legs, no load, nothing to protect against
// gravity yet).
#pragma once

#include <cstdint>

class LinkWatchdog {
public:
    explicit LinkWatchdog(float timeoutS);

    // Call whenever a packet with sequence number `seq` is decoded and
    // passes the wraparound-safe freshness check. Updates "last seen"
    // only if seq is actually newer than the last accepted one -- a
    // stale/duplicate packet must not reset the watchdog, same
    // latest-wins rule applied to commands elsewhere in this project.
    // Returns whether it was accepted (counted as fresh).
    bool observePacket(uint32_t seq, float nowS);

    // Call every control-loop tick. True if it's been longer than the
    // configured timeout since the last accepted packet.
    bool hasTimedOut(float nowS) const;

    // Seconds since the last accepted packet, or -1 if none has arrived
    // yet.
    float secondsSinceLastPacket(float nowS) const;

    bool hasEverReceivedPacket() const { return hasReceived_; }

private:
    float timeoutS_;
    bool hasReceived_ = false;
    uint32_t lastSeq_ = 0;
    float lastSeenAtS_ = 0.0f;
};
