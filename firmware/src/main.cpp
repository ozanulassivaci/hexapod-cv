// Wiring lives here as each stage lands (order of work: watchdog+safe
// state, then I2C+single servo, then the profile store, then bench mode --
// see docs/protocol.md and the design discussion this was built from).
// Currently just enough to prove lib/core cross-compiles for the real
// target, not just native/host.
#include <Arduino.h>

#include "Config.h"
#include "LinkWatchdog.h"
#include "SafeState.h"
#include "SequenceGuard.h"

void setup() {
    Serial.begin(SERIAL_BAUD);
}

void loop() {
}
