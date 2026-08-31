// JSON wire codec matching docs/protocol.md exactly. Pure -- ArduinoJson
// works unmodified on native/host, so this is unit tested the same way as
// everything else in lib/core.
//
// Command is a tagged union (one struct, only the fields for `type` are
// meaningful) rather than a class hierarchy -- simpler for embedded C++,
// no heap allocation, no virtual dispatch, and the struct is small enough
// (~600 bytes) that using it as a transient stack value per packet costs
// nothing on an ESP32-S3.
//
// walk/turn/stop/body_height/pan_tilt/face decode and validate fully, and
// are acknowledged (ok=true), but have no effect on hardware this build --
// there is no gait/IK to act on them yet (CLAUDE.md: this session is
// explicitly scoped to bench-testing plumbing only). They still update
// MotionState so last_applied reports honestly.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Config.h"
#include "ServoProfile.h"

enum class CommandType {
    Walk,
    Turn,
    Stop,
    BodyHeight,
    PanTilt,
    Face,
    Calibrate,
    CalibrationMode,
    ReadOffsets,
    WriteOffsets,
    BenchMode,
    BenchPulse,
    RecordLimit,
    BenchHealthNote,
    Ping,
    Unknown,
};

struct OffsetEntry {
    uint8_t servoIndex = 0;
    int16_t offsetUs = 0;
    int8_t sign = 1;
};

struct Command {
    CommandType type = CommandType::Unknown;
    uint32_t seq = 0;

    // walk
    float vx = 0.0f;
    float vy = 0.0f;
    // turn
    float rate = 0.0f;
    // walk / turn
    float speed = 0.0f;
    // body_height
    float height = 0.0f;
    // pan_tilt
    float pan = 0.0f;
    float tilt = 0.0f;
    // face
    char mood[16] = {0};

    // calibrate / record_limit / bench_health_note
    uint8_t servoIndex = 0;
    // calibrate
    int16_t offsetUs = 0;
    int8_t sign = 1;
    // calibration_mode / bench_mode
    bool armed = false;
    // write_offsets
    OffsetEntry offsetEntries[SERVO_COUNT];
    uint8_t offsetEntryCount = 0;
    // bench_pulse
    uint8_t board = 0;
    uint8_t channel = 0;
    uint16_t pulseUs = 0;
    bool hasServoIndex = false;  // bench_pulse's optional servo_index
    // record_limit
    bool boundIsMax = false;  // false = min
    // bench_health_note
    char note[HEALTH_NOTE_MAX_LEN + 1] = {0};
};

struct DecodeResult {
    bool ok = false;
    Command command;
    char error[80] = {0};
};

DecodeResult decodeCommand(const uint8_t* data, size_t len);

// --- Telemetry (robot -> PC) ----------------------------------------------

// Only the fields for a Walk/Turn/Stop/BodyHeight/PanTilt/Face command are
// meaningful -- Command is reused here just for its motion-field shape,
// not its calibrate/bench fields.
struct MotionState {
    CommandType type = CommandType::Stop;
    float vx = 0.0f, vy = 0.0f, rate = 0.0f, speed = 0.0f, height = 0.0f;
    float pan = 0.0f, tilt = 0.0f;
    char mood[16] = {0};
};

struct Telemetry {
    uint32_t seqEcho = 0;
    bool ok = true;
    char error[80] = {0};  // empty string = null on the wire
    uint32_t faultFlags = 0;
    bool hasGaitPhase = false;
    float gaitPhase = 0.0f;
    bool hasRailMv = false;
    int32_t railMv = 0;
    float linkTimeoutS = 0.0f;
    bool calibrationArmed = false;
    bool benchArmed = false;
    bool robotAssembled = false;  // echoes Config.h's compiled-in ROBOT_ASSEMBLED, see SafeState.h
    // Cumulative since GaitEngine construction (never since the last
    // packet) -- mirrors transport/protocol.py's Telemetry, see that
    // struct's comment for the full rationale.
    uint32_t ikClipCount = 0;
    float ikClipWorstMm = 0.0f;
    uint32_t jointClipCount = 0;
    float jointClipWorstDeg = 0.0f;
    bool hasLastApplied = false;
    MotionState lastApplied;
    bool hasProfiles = false;
    ServoProfile profiles[SERVO_COUNT];
};

// Writes JSON into outBuf (NOT null-terminated beyond what snprintf-style
// buffers guarantee -- use the returned length, don't strlen()). Returns 0
// if it didn't fit.
size_t encodeTelemetry(const Telemetry& telemetry, uint8_t* outBuf, size_t outBufLen);
