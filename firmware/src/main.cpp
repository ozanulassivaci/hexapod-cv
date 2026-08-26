// Wiring lives here as each stage lands (order of work: watchdog+safe
// state, then I2C+single servo, then the profile store, then bench mode --
// see docs/protocol.md and the design discussion this was built from).
//
// Current stage: I2C + PCA9685 init, wired to safe state. No UDP/serial
// command path yet (that's the profile-store and bench-mode stages) --
// with nothing able to arrive, the watchdog stays permanently timed-out
// by design (see LinkWatchdog.h), so every loop() tick keeps confirming
// safe state. This is the correct, boring behavior for this stage: prove
// I2C/PCA9685 bring-up works without yet being able to command anything
// away from released.
#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "LinkWatchdog.h"
#include "NvsServoProfileStore.h"
#include "Pca9685ServoOutput.h"
#include "SafeState.h"

static Pca9685ServoOutput servoOutput;
static NvsServoProfileStore profileStore;
static LinkWatchdog watchdog(HX_LINK_TIMEOUT_S);

void setup() {
    Serial.begin(SERIAL_BAUD);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    bool pca9685Ok = servoOutput.begin();
    if (!pca9685Ok) {
        Serial.println("PCA9685 init failed on at least one board -- check "
                        "I2C wiring/address jumpers before trusting anything "
                        "past this point.");
    }

    profileStore.begin();

    enterSafeState(servoOutput);  // redundant with begin()'s own release-all; cheap, explicit
    Serial.println("boot: safe state (all channels released), watchdog untripped-since-boot");
}

void loop() {
    float nowS = millis() / 1000.0f;

    if (watchdog.hasTimedOut(nowS)) {
        enterSafeState(servoOutput);
    }

    delay(CONTROL_LOOP_INTERVAL_MS);
}
