#include "LinkWatchdog.h"

#include "SequenceGuard.h"

LinkWatchdog::LinkWatchdog(float timeoutS) : timeoutS_(timeoutS) {}

Observation LinkWatchdog::observePacket(uint32_t seq, float nowS) {
    Observation result = Observation::Accepted;
    if (hasReceived_) {
        // hasTimedOut() itself, not a re-derived comparison, so "the
        // watchdog considers the link down" and "the next packet
        // re-baselines" cannot drift apart.
        if (hasTimedOut(nowS)) {
            result = Observation::NewSession;
        } else if (!sequenceIsNewer(seq, lastSeq_)) {
            ++staleDropCount_;
            return Observation::Stale;
        }
    }
    lastSeq_ = seq;
    lastSeenAtS_ = nowS;
    hasReceived_ = true;
    return result;
}

bool LinkWatchdog::hasTimedOut(float nowS) const {
    if (!hasReceived_) {
        return true;
    }
    return (nowS - lastSeenAtS_) >= timeoutS_;
}

float LinkWatchdog::secondsSinceLastPacket(float nowS) const {
    if (!hasReceived_) {
        return -1.0f;
    }
    return nowS - lastSeenAtS_;
}
