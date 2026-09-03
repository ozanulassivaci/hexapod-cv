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

// Outcome of one inbound packet. Three states, not a bool: a re-baseline
// is a real state change worth logging and counting, and collapsing it
// into "accepted" is what made the original bug's recovery path
// unobservable. Mirrored in transport/link_watchdog.py's Observation.
enum class Observation {
    Accepted,
    Stale,
    NewSession,
};

class LinkWatchdog {
public:
    explicit LinkWatchdog(float timeoutS);

    // Call whenever a packet with sequence number `seq` is decoded.
    // Latest-wins freshness -- a stale/duplicate packet must not reset
    // the watchdog, the same rule applied to commands elsewhere in this
    // project -- with one exception: if the link has already been silent
    // longer than the timeout, the next packet is accepted regardless of
    // its sequence number and becomes the new baseline (NewSession).
    //
    // That exception exists because the plain rule assumed one PC
    // counter for the life of this boot. Restart the GUI without
    // power-cycling the robot and the new process starts at seq 0, which
    // is "older" than whatever the previous session reached -- so every
    // packet it will ever send is rejected, silently, until its counter
    // climbs past the old one. At the 10Hz heartbeat and a one-minute
    // previous session that is a full minute of dead link.
    //
    // Re-baselining is safe precisely where it triggers: the link is
    // already past its timeout, so the failsafe has already fired and
    // the robot is already in safe state. A peer arriving after that is
    // a new session by definition. Inside an active stream -- what the
    // rule actually exists for -- reordering and duplicates are rejected
    // exactly as before.
    Observation observePacket(uint32_t seq, float nowS);

    // Call every control-loop tick. True if it's been longer than the
    // configured timeout since the last accepted packet.
    bool hasTimedOut(float nowS) const;

    // Seconds since the last accepted packet, or -1 if none has arrived
    // yet.
    float secondsSinceLastPacket(float nowS) const;

    bool hasEverReceivedPacket() const { return hasReceived_; }

    // The last sequence number actually accepted as fresh. Only
    // meaningful once hasEverReceivedPacket() is true. Exposed for
    // diagnostics -- a packet rejected by observePacket() is otherwise
    // indistinguishable from one that never arrived, and "which sequence
    // did you compare mine against" is the one fact that tells those
    // apart from the PC side.
    uint32_t lastAcceptedSeq() const { return lastSeq_; }

    // Packets rejected by the freshness check since boot. Cumulative and
    // never reset -- docs/protocol.md Section 4's "drop counter kept for
    // observability", which the spec claimed existed long before it did.
    // A nonzero and climbing value while the GUI shows no link is the
    // single clearest signal that packets are arriving and being
    // discarded rather than never arriving at all.
    uint32_t staleDropCount() const { return staleDropCount_; }

private:
    float timeoutS_;
    bool hasReceived_ = false;
    uint32_t lastSeq_ = 0;
    float lastSeenAtS_ = 0.0f;
    uint32_t staleDropCount_ = 0;
};
