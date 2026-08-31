// Order of work this firmware was built in (see docs/protocol.md and the
// design discussion this was built from): watchdog+safe state, I2C+single
// servo, the profile store, bench mode. This file is the final wiring --
// UDP listener, serial mirror, WiFi/AP fallback, OTA -- around the pure
// logic in lib/core, which is what's actually unit tested. This file
// itself is orchestration, deliberately thin, and can only be verified by
// real cross-compilation plus reading the exact library APIs it calls
// (ArduinoOTA, ESP32 core WiFi) -- there is no host-side way to test
// hardware/network wiring, and no physical hardware available this
// session to run it against.
//
// Single-task, cooperative design: UDP (WiFiUDP, polled, not AsyncUDP),
// serial, and ArduinoOTA.handle() are all serviced from this one loop() on
// one task. No mutexes anywhere in the safety-relevant state (arm gates,
// dwell guard, watchdog) because there is only ever one thread of
// execution touching it -- this is the single biggest lever against the
// feature-interaction bugs the design discussion's matrix was about.
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <cstring>

#include "AngleToPulse.h"
#include "ArmGate.h"
#include "Config.h"
#include "DwellGuard.h"
#include "Gait.h"
#include "GatedServoDriver.h"
#include "Kinematics.h"
#include "LinkWatchdog.h"
#include "NvsServoProfileStore.h"
#include "Pca9685ServoOutput.h"
#include "Protocol.h"
#include "SafeState.h"
#include "ServoMap.h"
#include "ServoProfile.h"
#include "WifiSetup.h"
#include "secrets.h"

// --- shared state ----------------------------------------------------

static Pca9685ServoOutput servoOutput;
static NvsServoProfileStore profileStore;
static GatedServoDriver gatedDriver(servoOutput, profileStore);

static LinkWatchdog linkWatchdog(HX_LINK_TIMEOUT_S);
static ArmGate calibrationGate(HX_CALIBRATION_ARM_TIMEOUT_S);
static ArmGate benchGate(HX_BENCH_ARM_TIMEOUT_S);
static DwellGuard dwellGuard(DWELL_TIMEOUT_S);
static GaitEngine gaitEngine;
static uint32_t lastGaitMillis = 0;

// Selected once, at compile time, from Config.h's ROBOT_ASSEMBLED build
// flag -- see SafeState.h for the full argument for why this is a build
// flag and not a runtime command or an inference from recorded profiles.
static const SafetyMode kSafetyMode = ROBOT_ASSEMBLED ? SafetyMode::Assembled : SafetyMode::Bench;

static WifiSetup wifiSetup;
static WiFiUDP udp;
static IPAddress pcAddr;
static uint16_t pcPort = 0;
static bool havePcAddr = false;

static MotionState lastMotion;
static bool hasMotion = false;
static uint32_t faultFlags = 0;

// Gait's actual inputs -- deliberately separate from lastMotion, which
// stays a plain echo of whichever single command was processed most
// recently (docs/protocol.md Section 3's "compact echo of the currently-
// active command", a debugging aid). lastMotion.height would get reset
// to 0 by the very next resent walk/turn packet if gait read height from
// it directly (the GUI's heartbeat resends walk/turn ~10x/s; body_height
// is sent once, on slider change, not resent) -- these four/one persist
// independently per axis instead, updated only by the command type that
// actually carries them.
static float currentVx = 0.0f;
static float currentVy = 0.0f;
static float currentSpeed = 0.0f;
static float currentRotation = 0.0f;
static float currentBodyHeight = kDefaultBodyHeight;

static bool otaInProgress = false;

// --- OTA gate ------------------------------------------------------------
//
// ArduinoOTA's onStart callback has no way to refuse an update (it's
// void(void), a notification, not a gate -- confirmed by reading the
// library header, not assumed). The mechanism that actually works: only
// call ArduinoOTA.handle() when it's safe to; skipping it entirely means
// the device never services OTA discovery/upload requests, which is what
// "refused" has to mean given the library's actual API.

static bool otaAllowed() {
    if (calibrationGate.isArmed()) return false;
    if (benchGate.isArmed()) return false;
    for (uint8_t ch = 0; ch < PCA9685_CHANNELS_PER_BOARD; ++ch) {
        if (dwellGuard.isAway(PCA9685_ADDR_BOARD_A, ch)) return false;
        if (dwellGuard.isAway(PCA9685_ADDR_BOARD_B, ch)) return false;
    }
    // Gait is real now (it wasn't when this gate was first built) -- an
    // OTA flash mid-walk would otherwise freeze the control loop
    // (loop() returns early while otaInProgress) with no warning.
    if (!isMotionIdle(currentVx, currentVy, currentSpeed, currentRotation)) return false;
    return true;
}

static void onOtaStart() {
    otaInProgress = true;
    enterSafeState(servoOutput, kSafetyMode);  // matches the design's other safety-state triggers
    Serial.println("OTA: update starting");
}

static void onOtaEnd() {
    Serial.println("OTA: update complete, rebooting");
    otaInProgress = false;
}

static void onOtaError(ota_error_t error) {
    Serial.printf("OTA: failed (code %d), resuming normal operation\n", error);
    otaInProgress = false;
}

// --- shared command handling (UDP and serial both funnel through this) --

static void handleCommand(const Command& cmd, uint32_t seq, Telemetry& out) {
    float nowS = millis() / 1000.0f;
    out.seqEcho = seq;
    out.ok = true;
    out.error[0] = '\0';

    switch (cmd.type) {
        case CommandType::Walk:
        case CommandType::Turn:
        case CommandType::Stop:
        case CommandType::BodyHeight:
        case CommandType::PanTilt:
        case CommandType::Face:
            // last_applied always echoes the single most recent command,
            // whatever it was -- a debugging aid (docs/protocol.md
            // Section 3), unrelated to what actually drives gait below.
            lastMotion.type = cmd.type;
            lastMotion.vx = cmd.vx;
            lastMotion.vy = cmd.vy;
            lastMotion.rate = cmd.rate;
            lastMotion.speed = cmd.speed;
            lastMotion.height = cmd.height;
            lastMotion.pan = cmd.pan;
            lastMotion.tilt = cmd.tilt;
            std::strncpy(lastMotion.mood, cmd.mood, sizeof(lastMotion.mood) - 1);
            hasMotion = true;

            // Gait's actual inputs -- each axis updated only by the
            // command type that actually carries it, so e.g. a walk
            // command doesn't reset body height back to whatever a
            // never-sent body_height field defaults to (see the
            // currentVx/.../currentBodyHeight declaration comment).
            switch (cmd.type) {
                case CommandType::Walk:
                    currentVx = cmd.vx;
                    currentVy = cmd.vy;
                    currentSpeed = cmd.speed;
                    currentRotation = 0.0f;
                    break;
                case CommandType::Turn:
                    currentVx = 0.0f;
                    currentVy = 0.0f;
                    currentSpeed = cmd.speed;
                    currentRotation = cmd.rate;
                    break;
                case CommandType::Stop:
                    currentVx = 0.0f;
                    currentVy = 0.0f;
                    currentSpeed = 0.0f;
                    currentRotation = 0.0f;
                    break;
                case CommandType::BodyHeight:
                    currentBodyHeight = cmd.height;
                    break;
                default:
                    break;  // PanTilt/Face don't touch any gait axis
            }
            break;

        case CommandType::CalibrationMode:
            if (cmd.armed) {
                // Only arm on the false->true edge. A resent/duplicate
                // armed=true (indistinguishable on the wire from a
                // deliberate re-click, since the transport resends
                // whatever was last sent) must not indefinitely extend
                // the window just by continuing to arrive -- that would
                // defeat "auto-disarms if the operator walks away", since
                // the heartbeat would keep the gate open forever with the
                // operator having done nothing since the first arm.
                if (!calibrationGate.isArmed()) calibrationGate.arm(nowS);
            } else {
                calibrationGate.disarm();
            }
            break;

        case CommandType::Calibrate: {
            if (!calibrationGate.isArmed()) {
                out.ok = false;
                std::strncpy(out.error, "calibration not armed", sizeof(out.error) - 1);
                break;
            }
            ServoProfile existing = profileStore.get(cmd.servoIndex);
            profileStore.set(cmd.servoIndex, mergeOffsetSign(existing, cmd.offsetUs, cmd.sign));
            calibrationGate.refresh(nowS);
            break;
        }

        case CommandType::WriteOffsets: {
            if (!calibrationGate.isArmed()) {
                out.ok = false;
                std::strncpy(out.error, "calibration not armed", sizeof(out.error) - 1);
                break;
            }
            for (uint8_t i = 0; i < cmd.offsetEntryCount; ++i) {
                const OffsetEntry& e = cmd.offsetEntries[i];
                ServoProfile existing = profileStore.get(e.servoIndex);
                profileStore.set(e.servoIndex, mergeOffsetSign(existing, e.offsetUs, e.sign));
            }
            calibrationGate.refresh(nowS);
            break;
        }

        case CommandType::ReadOffsets: {
            out.hasProfiles = true;
            for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
                out.profiles[i] = profileStore.get(i);
            }
            break;
        }

        case CommandType::BenchMode:
            if (cmd.armed) {
                if (!benchGate.isArmed()) {
                    // docs/protocol.md Section 8's pre-announced TODO,
                    // now buildable: bench mode and gait must be
                    // mutually exclusive, since bench_pulse drives a
                    // channel directly with no kinematics in the way,
                    // and gait re-commands the same channels every tick
                    // otherwise. Refuse the arm request here (gait keeps
                    // running); gait itself stops ticking once armed
                    // (see loop()'s gaitShouldRun) -- two sides of the
                    // same exclusion, checked at different moments.
                    if (!isMotionIdle(currentVx, currentVy, currentSpeed, currentRotation)) {
                        out.ok = false;
                        std::strncpy(out.error, "gait active, cannot arm bench mode", sizeof(out.error) - 1);
                        break;
                    }
                    benchGate.arm(nowS);  // same edge-only reasoning as calibration_mode
                }
            } else {
                benchGate.disarm();
            }
            break;

        case CommandType::BenchPulse: {
            if (!benchGate.isArmed()) {
                out.ok = false;
                std::strncpy(out.error, "bench mode not armed", sizeof(out.error) - 1);
                break;
            }
            const char* reason = nullptr;
            bool applied = gatedDriver.commandPulse(cmd.board, cmd.channel, cmd.pulseUs,
                                                      cmd.hasServoIndex, cmd.servoIndex, &reason);
            if (!applied) {
                out.ok = false;
                std::strncpy(out.error, reason, sizeof(out.error) - 1);
                break;
            }
            benchGate.refresh(nowS);
            bool mustRelease = dwellGuard.observe(cmd.board, cmd.channel, cmd.pulseUs, nowS);
            if (mustRelease) {
                gatedDriver.release(cmd.board, cmd.channel);
                dwellGuard.reset(cmd.board, cmd.channel);
                out.ok = false;
                std::strncpy(out.error, "dwell timeout: released", sizeof(out.error) - 1);
            }
            break;
        }

        case CommandType::RecordLimit: {
            if (!benchGate.isArmed()) {
                out.ok = false;
                std::strncpy(out.error, "bench mode not armed", sizeof(out.error) - 1);
                break;
            }
            ServoProfile existing = profileStore.get(cmd.servoIndex);
            ServoProfile updated;
            LimitBound bound = cmd.boundIsMax ? LimitBound::Max : LimitBound::Min;
            if (!mergeLimit(existing, bound, cmd.pulseUs, updated)) {
                out.ok = false;
                std::strncpy(out.error, "min_pulse_us must be < max_pulse_us", sizeof(out.error) - 1);
                break;
            }
            profileStore.set(cmd.servoIndex, updated);
            benchGate.refresh(nowS);
            break;
        }

        case CommandType::BenchHealthNote: {
            // Not gated -- advisory record-keeping, not an actuation or
            // safety-relevant value. Doesn't refresh bench's window either.
            ServoProfile existing = profileStore.get(cmd.servoIndex);
            profileStore.set(cmd.servoIndex, mergeNote(existing, cmd.note));
            break;
        }

        case CommandType::Ping:
        default:
            break;
    }
}

// --- gait output -----------------------------------------------------

// Converts the gait engine's current per-leg joint angles into pulses
// and commands them through GatedServoDriver -- the only path any
// firmware code may use to command a pulse, so gait output is enforced
// against a servo's bench-recorded limit exactly like a bench_pulse
// command is, no separate/bypassable path. Returns true if any pulse was
// refused (surfaced as FAULT_SERVO_FAULT_BIT by the caller) --
// ANALYSIS.md Section 5.7's "reachability clamping is silent" gap,
// closed for the case that now matters: a specific servo's own verified-
// safe range, not just the generic protocol-level bound.
static bool driveGaitOutputs() {
    struct JointOutput {
        JointType joint;
        float servoDeg;
        float neutralServoDeg;
        uint16_t neutralPulseUs;
    };

    bool anyRefused = false;
    for (uint8_t legIndex = 0; legIndex < 6; ++legIndex) {
        ServoDeg servoDeg = toServoDeg(gaitEngine.state().legAngles[legIndex]);
        const JointOutput joints[3] = {
            {JointType::Coxa, servoDeg.coxaDeg, 90.0f, NEUTRAL_PULSE_US},
            {JointType::Femur, servoDeg.femurDeg, 90.0f, NEUTRAL_PULSE_US},
            {JointType::Tibia, servoDeg.tibiaDeg, 0.0f, TIBIA_NEUTRAL_PULSE_US},
        };
        for (const JointOutput& j : joints) {
            uint8_t servoIndex = servoIndexFor(legIndex, j.joint);
            const ServoMapEntry& entry = kServoMap[servoIndex];
            ServoProfile profile = profileStore.get(servoIndex);
            uint16_t pulseUs =
                angleToPulseUs(j.servoDeg, j.neutralServoDeg, j.neutralPulseUs, profile.sign, profile.offsetUs);
            const char* reason = nullptr;
            bool applied = gatedDriver.commandPulse(entry.board, entry.channel, pulseUs, true, servoIndex, &reason);
            if (!applied) anyRefused = true;
        }
    }
    return anyRefused;
}

static void sendTelemetry(const Telemetry& t) {
    if (!havePcAddr) return;
    uint8_t buf[4096];
    size_t len = encodeTelemetry(t, buf, sizeof(buf));
    if (len == 0) return;
    udp.beginPacket(pcAddr, pcPort);
    udp.write(buf, len);
    udp.endPacket();
}

static void buildBaseTelemetry(Telemetry& t) {
    t.faultFlags = faultFlags;
    t.linkTimeoutS = HX_LINK_TIMEOUT_S;
    t.calibrationArmed = calibrationGate.isArmed();
    t.benchArmed = benchGate.isArmed();
    t.robotAssembled = ROBOT_ASSEMBLED;
    t.hasLastApplied = hasMotion;
    if (hasMotion) t.lastApplied = lastMotion;
    // null while not walking (docs/protocol.md Section 3), matching
    // isMotionIdle -- the same "idle" definition footTarget()/the
    // bench-arm-refusal check use.
    t.hasGaitPhase = !isMotionIdle(currentVx, currentVy, currentSpeed, currentRotation);
    t.gaitPhase = gaitEngine.state().phase;
    t.ikClipCount = gaitEngine.state().ikClipCount;
    t.ikClipWorstMm = gaitEngine.state().ikClipWorstMm;
    t.jointClipCount = gaitEngine.state().jointClipCount;
    t.jointClipWorstDeg = gaitEngine.state().jointClipWorstDeg;
}

// --- UDP -----------------------------------------------------------------

static void processUdp() {
    int packetSize;
    while ((packetSize = udp.parsePacket()) > 0) {
        static uint8_t buf[UDP_RECV_BUFFER_SIZE];
        int len = udp.read(buf, sizeof(buf) - 1);
        if (len <= 0) continue;

        pcAddr = udp.remoteIP();
        pcPort = udp.remotePort();
        havePcAddr = true;

        DecodeResult decoded = decodeCommand(buf, static_cast<size_t>(len));
        if (!decoded.ok) {
            continue;  // malformed packet dropped without crashing, no reply -- nothing valid to echo a seq for
        }

        bool fresh = linkWatchdog.observePacket(decoded.command.seq, millis() / 1000.0f);
        if (!fresh) {
            continue;  // stale/duplicate, latest-wins -- silently dropped, not an error
        }
        faultFlags &= ~FAULT_LINK_TIMEOUT_BIT;

        Telemetry telemetry;
        buildBaseTelemetry(telemetry);
        handleCommand(decoded.command, decoded.command.seq, telemetry);
        buildBaseTelemetry(telemetry);  // re-sync armed/last_applied after handling
        telemetry.seqEcho = decoded.command.seq;
        sendTelemetry(telemetry);
    }
}

// --- serial mirror -------------------------------------------------------
//
// Plain-text, not the JSON wire protocol -- meant for typing by hand in a
// terminal, diagnostics and bench testing without WiFi. Still carries the
// same USB+servo-rail risk as anything else that moves a servo; see
// docs/HOW_TO_USE.md. Minimal grammar, not a full alternate control
// surface: status, cal arm/disarm, bench arm/disarm, bench pulse <board>
// <channel> <pulse_us>, bench release <board> <channel>, ping.

static void printStatus() {
    Serial.print("wifi=");
    Serial.print(wifiSetup.mode() == WifiMode::Station ? "station" : "ap-fallback");
    Serial.print(" link_timed_out=");
    Serial.print(linkWatchdog.hasTimedOut(millis() / 1000.0f) ? "yes" : "no");
    Serial.print(" cal_armed=");
    Serial.print(calibrationGate.isArmed() ? "yes" : "no");
    Serial.print(" bench_armed=");
    Serial.print(benchGate.isArmed() ? "yes" : "no");
    Serial.print(" safety_mode=");
    Serial.print(ROBOT_ASSEMBLED ? "assembled" : "bench");
    Serial.print(" gait_phase=");
    Serial.println(gaitEngine.state().phase);
}

static void processSerialLine(String line) {
    line.trim();
    if (line.length() == 0) return;

    if (line == "status") {
        printStatus();
        return;
    }
    if (line == "ping") {
        Serial.println("pong");
        return;
    }
    if (line == "cal arm") {
        if (!calibrationGate.isArmed()) calibrationGate.arm(millis() / 1000.0f);
        Serial.println("calibration armed");
        return;
    }
    if (line == "cal disarm") {
        calibrationGate.disarm();
        Serial.println("calibration disarmed");
        return;
    }
    if (line == "bench arm") {
        if (!benchGate.isArmed()) benchGate.arm(millis() / 1000.0f);
        Serial.println("bench armed");
        return;
    }
    if (line == "bench disarm") {
        benchGate.disarm();
        Serial.println("bench disarmed");
        return;
    }
    if (line.startsWith("bench pulse ")) {
        int board, channel, pulseUs;
        if (sscanf(line.c_str(), "bench pulse %d %d %d", &board, &channel, &pulseUs) == 3) {
            if (!benchGate.isArmed()) {
                Serial.println("rejected: bench mode not armed");
                return;
            }
            const char* reason = nullptr;
            bool applied = gatedDriver.commandPulse(static_cast<uint8_t>(board), static_cast<uint8_t>(channel),
                                                      static_cast<uint16_t>(pulseUs), false, 0, &reason);
            if (!applied) {
                Serial.print("rejected: ");
                Serial.println(reason);
                return;
            }
            benchGate.refresh(millis() / 1000.0f);
            dwellGuard.observe(static_cast<uint8_t>(board), static_cast<uint8_t>(channel),
                                static_cast<uint16_t>(pulseUs), millis() / 1000.0f);
            Serial.println("ok");
        } else {
            Serial.println("usage: bench pulse <board> <channel> <pulse_us>");
        }
        return;
    }
    if (line.startsWith("bench release ")) {
        int board, channel;
        if (sscanf(line.c_str(), "bench release %d %d", &board, &channel) == 2) {
            gatedDriver.release(static_cast<uint8_t>(board), static_cast<uint8_t>(channel));
            dwellGuard.reset(static_cast<uint8_t>(board), static_cast<uint8_t>(channel));
            Serial.println("ok");
        } else {
            Serial.println("usage: bench release <board> <channel>");
        }
        return;
    }

    Serial.println("unknown command. try: status, ping, cal arm/disarm, bench arm/disarm, "
                    "bench pulse <board> <channel> <pulse_us>, bench release <board> <channel>");
}

static void processSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    processSerialLine(line);
}

// --- setup / loop ----------------------------------------------------

void setup() {
    Serial.begin(SERIAL_BAUD);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!servoOutput.begin()) {
        Serial.println("PCA9685 init failed on at least one board -- check "
                        "I2C wiring/address jumpers before trusting anything past this point.");
    }
    profileStore.begin();
    gatedDriver.releaseAll();  // boot state = failsafe state, unconditionally, in both SafetyModes -- see SafeState.h

    wifiSetup.begin();

    udp.begin(ROBOT_UDP_PORT);

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart(onOtaStart);
    ArduinoOTA.onEnd(onOtaEnd);
    ArduinoOTA.onError(onOtaError);
    ArduinoOTA.begin();

    lastGaitMillis = millis();

    Serial.print("boot: safe state (all channels released), watchdog untripped-since-boot, safety_mode=");
    Serial.println(ROBOT_ASSEMBLED ? "assembled (hold on fault)" : "bench (release on fault)");
}

void loop() {
    if (otaInProgress) {
        ArduinoOTA.handle();
        return;  // nothing else runs during an active flash -- see the OTA gate note above
    }
    if (otaAllowed()) {
        ArduinoOTA.handle();
    }
    // else: OTA discovery/upload requests go unanswered this tick -- the
    // only way to "refuse" given ArduinoOTA's onStart has no reject path.

    if (wifiSetup.maintain()) {
        enterSafeState(servoOutput, kSafetyMode);  // AP-fallback transition -- matches the watchdog's own response
    }

    float nowS = millis() / 1000.0f;
    calibrationGate.tick(nowS);
    benchGate.tick(nowS);

    if (linkWatchdog.hasTimedOut(nowS)) {
        enterSafeState(servoOutput, kSafetyMode);
        calibrationGate.disarm();
        benchGate.disarm();
        faultFlags |= FAULT_LINK_TIMEOUT_BIT;
    }

    processUdp();
    processSerial();

    // Gait runs only when the link is healthy and bench mode isn't
    // fighting it over the same channels -- not ticking GaitEngine at
    // all is what "frozen"/"suspended" means here (see Gait.h,
    // docs/protocol.md Section 8). Re-checks fresh state (a packet
    // processed just above may have cleared the timeout or changed
    // bench_armed).
    uint32_t nowMillis = millis();
    float dtS = (nowMillis - lastGaitMillis) / 1000.0f;
    lastGaitMillis = nowMillis;
    bool gaitShouldRun = !linkWatchdog.hasTimedOut(nowS) && !benchGate.isArmed();
    if (gaitShouldRun) {
        gaitEngine.tick(dtS, currentVx, currentVy, currentSpeed, currentRotation, currentBodyHeight);
        bool anyRefused = driveGaitOutputs();
        if (anyRefused) {
            faultFlags |= FAULT_SERVO_FAULT_BIT;
        } else {
            faultFlags &= ~FAULT_SERVO_FAULT_BIT;
        }
        // Level-triggered on the tick that just ran -- the cumulative
        // ikClipCount/jointClipCount in telemetry are the diagnostic
        // detail behind this bit, not what it's derived from. Today gait
        // is the only mode, and the verified-safe envelope means any
        // clip here is genuinely anomalous (see GaitState's docstring in
        // Gait.h) -- once Test Leg exploration mode exists, that code
        // needs to stop setting this bit while exploration is deliberate.
        if (gaitEngine.state().clippedThisTick) {
            faultFlags |= FAULT_IK_CLIP_BIT;
        } else {
            faultFlags &= ~FAULT_IK_CLIP_BIT;
        }
    }

    delay(CONTROL_LOOP_INTERVAL_MS);
}
