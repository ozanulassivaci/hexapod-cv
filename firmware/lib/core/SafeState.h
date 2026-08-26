// The one function that defines what "safe" means in this firmware --
// called from every trigger that needs it (link watchdog firing,
// AP-fallback entry, OTA start), so there is exactly one definition of
// "safe" in the codebase, not several that could disagree.
//
// Two modes, selected once at startup from Config.h's ROBOT_ASSEMBLED
// (a build flag, not a runtime command or an inference from recorded
// servo profiles -- see the design discussion this was built from for
// the full argument: the two misconfigurations are not symmetric, a
// build flag that defaults to Bench is sticky and requires a deliberate
// reflash to change, and a fully bench-tested profile table is not
// evidence a leg is on the chassis right now):
//
// - Bench: release every channel (detach, no PWM). Correct when there
//   are no legs bearing weight -- releasing costs nothing and is
//   strictly safer than continuing to fight for a possibly-stressed
//   commanded position once we can no longer verify anyone is watching.
// - Assembled: hold, not release or re-pose. Releasing a loaded joint
//   means it gives way under the robot's own weight; commanding new
//   motion during a fault is exactly what reference/ANALYSIS.md Section
//   5.4 flags as wrong (a fault might BE a bad gait target -- moving
//   further during it compounds the failure instead of freezing it).
//   "Hold" costs nothing to implement here because it's the literal
//   absence of an action: main.cpp's control loop simply does not call
//   GaitEngine::tick() or GatedServoDriver::commandPulse() while unsafe
//   (see loop()'s gaitShouldRun), so the PCA9685 keeps outputting
//   whatever it was last told with zero further firmware involvement --
//   this function's Assembled branch has nothing to do, and says so.
#pragma once

#include "ServoOutput.h"

enum class SafetyMode { Bench, Assembled };

void enterSafeState(ServoOutput& output, SafetyMode mode);
