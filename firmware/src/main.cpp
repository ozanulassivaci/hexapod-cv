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

#include "ArmGate.h"
#include "Config.h"
#include "DwellGuard.h"
#include "GatedServoDriver.h"
#include "LinkWatchdog.h"
#include "NvsServoProfileStore.h"
#include "Pca9685ServoOutput.h"
#include "Protocol.h"
#include "SafeState.h"
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

static WifiSetup wifiSetup;
static WiFiUDP udp;
static IPAddress pcAddr;
static uint16_t pcPort = 0;
static bool havePcAddr = false;

static MotionState lastMotion;
static bool hasMotion = false;
static uint32_t faultFlags = 0;

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
    return true;
}

static void onOtaStart() {
    otaInProgress = true;
    gatedDriver.releaseAll();  // defensive -- matches the design's other safety-state triggers
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
            // No gait/IK this build -- acknowledged and recorded for
            // last_applied, no hardware effect. See CLAUDE.md: this
            // session is scoped to bench-testing plumbing only.
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
                if (!benchGate.isArmed()) benchGate.arm(nowS);  // same edge-only reasoning as calibration_mode
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
    t.hasLastApplied = hasMotion;
    if (hasMotion) t.lastApplied = lastMotion;
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
    Serial.println(benchGate.isArmed() ? "yes" : "no");
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
    gatedDriver.releaseAll();

    wifiSetup.begin();

    udp.begin(ROBOT_UDP_PORT);

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart(onOtaStart);
    ArduinoOTA.onEnd(onOtaEnd);
    ArduinoOTA.onError(onOtaError);
    ArduinoOTA.begin();

    Serial.println("boot: safe state (all channels released), watchdog untripped-since-boot");
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
        gatedDriver.releaseAll();  // AP-fallback transition -- defensive, matches the watchdog's own response
    }

    float nowS = millis() / 1000.0f;
    calibrationGate.tick(nowS);
    benchGate.tick(nowS);

    if (linkWatchdog.hasTimedOut(nowS)) {
        gatedDriver.releaseAll();
        calibrationGate.disarm();
        benchGate.disarm();
        faultFlags |= FAULT_LINK_TIMEOUT_BIT;
    }

    processUdp();
    processSerial();

    delay(CONTROL_LOOP_INTERVAL_MS);
}
