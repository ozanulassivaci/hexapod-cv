#include "LinkWatchdog.h"

#include "SequenceGuard.h"

LinkWatchdog::LinkWatchdog(float timeoutS) : timeoutS_(timeoutS) {}

bool LinkWatchdog::observePacket(uint32_t seq, float nowS) {
    if (hasReceived_ && !sequenceIsNewer(seq, lastSeq_)) {
        return false;
    }
    lastSeq_ = seq;
    lastSeenAtS_ = nowS;
    hasReceived_ = true;
    return true;
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
