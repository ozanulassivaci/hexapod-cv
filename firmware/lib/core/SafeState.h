// The one function that defines what "safe" means in this firmware --
// called from every trigger that needs it (link watchdog firing, dwell
// timeout, AP-fallback entry, OTA start), so there is exactly one
// definition of "safe" in the codebase, not several that could disagree.
//
// Release (detach, no PWM), not hold at any position. This is correct for
// this phase -- loose servos, no legs, nothing under load, so releasing
// costs nothing and is strictly safer than continuing to fight for a
// possibly-stressed commanded position once we can no longer verify
// anyone is watching. It is WRONG once a leg is attached and bearing
// weight: releasing a loaded joint means it gives way under the robot's
// own weight. Revisit this function -- deliberately, not by accident --
// once gait/IK/assembly exist and there is a real standing pose to hold
// instead.
#pragma once

#include "ServoOutput.h"

void enterSafeState(ServoOutput& output);
