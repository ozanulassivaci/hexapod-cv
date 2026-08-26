// Abstraction over persisting/loading a ServoProfile by servo_index, so
// the merge/gate logic that uses it is unit testable on the host against
// an in-memory fake, and only the real NVS-backed store needs actual
// hardware -- same pattern as ServoOutput.
#pragma once

#include "ServoProfile.h"

class ServoProfileStore {
public:
    virtual ~ServoProfileStore() = default;
    virtual ServoProfile get(uint8_t servoIndex) = 0;
    virtual void set(uint8_t servoIndex, const ServoProfile& profile) = 0;
};
