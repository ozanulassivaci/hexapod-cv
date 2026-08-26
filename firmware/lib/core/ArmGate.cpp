#include "ArmGate.h"

ArmGate::ArmGate(float timeoutS) : timeoutS_(timeoutS) {}

void ArmGate::arm(float nowS) {
    armed_ = true;
    armedUntilS_ = nowS + timeoutS_;
}

void ArmGate::disarm() {
    armed_ = false;
}

void ArmGate::refresh(float nowS) {
    if (armed_) {
        armedUntilS_ = nowS + timeoutS_;
    }
}

void ArmGate::tick(float nowS) {
    if (armed_ && nowS >= armedUntilS_) {
        armed_ = false;
    }
}
