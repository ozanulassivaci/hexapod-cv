#include "Protocol.h"

#include <ArduinoJson.h>
#include <cstdio>
#include <cstring>

namespace {

void setError(DecodeResult& result, const char* msg) {
    result.ok = false;
    std::strncpy(result.error, msg, sizeof(result.error) - 1);
    result.error[sizeof(result.error) - 1] = '\0';
}

bool getFloat(JsonVariantConst obj, const char* key, float& out) {
    JsonVariantConst v = obj[key];
    if (v.isNull() || v.is<bool>() || !v.is<float>()) return false;
    out = v.as<float>();
    return true;
}

bool getInt(JsonVariantConst obj, const char* key, long& out) {
    JsonVariantConst v = obj[key];
    if (v.isNull() || v.is<bool>() || !v.is<long>()) return false;
    out = v.as<long>();
    return true;
}

bool getBool(JsonVariantConst obj, const char* key, bool& out) {
    JsonVariantConst v = obj[key];
    if (v.isNull() || !v.is<bool>()) return false;
    out = v.as<bool>();
    return true;
}

bool getString(JsonVariantConst obj, const char* key, char* out, size_t outLen) {
    JsonVariantConst v = obj[key];
    if (v.isNull() || !v.is<const char*>()) return false;
    std::strncpy(out, v.as<const char*>(), outLen - 1);
    out[outLen - 1] = '\0';
    return true;
}

bool requireRangeFloat(DecodeResult& result, const char* name, float value, float lo, float hi) {
    if (!(value >= lo && value <= hi)) {  // NaN-safe: NaN compares false either way, correctly rejected
        char msg[80];
        snprintf(msg, sizeof(msg), "%s out of range", name);
        setError(result, msg);
        return false;
    }
    return true;
}

bool requireRangeInt(DecodeResult& result, const char* name, long value, long lo, long hi) {
    if (value < lo || value > hi) {
        char msg[80];
        snprintf(msg, sizeof(msg), "%s out of range", name);
        setError(result, msg);
        return false;
    }
    return true;
}

bool decodeWalk(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    if (!getFloat(obj, "vx", c.vx) || !getFloat(obj, "vy", c.vy) || !getFloat(obj, "speed", c.speed)) {
        setError(result, "missing or invalid walk field");
        return false;
    }
    return requireRangeFloat(result, "vx", c.vx, -1.0f, 1.0f) &&
           requireRangeFloat(result, "vy", c.vy, -1.0f, 1.0f) &&
           requireRangeFloat(result, "speed", c.speed, 0.0f, 100.0f);
}

bool decodeTurn(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    if (!getFloat(obj, "rate", c.rate) || !getFloat(obj, "speed", c.speed)) {
        setError(result, "missing or invalid turn field");
        return false;
    }
    return requireRangeFloat(result, "rate", c.rate, -1.0f, 1.0f) &&
           requireRangeFloat(result, "speed", c.speed, 0.0f, 100.0f);
}

bool decodeBodyHeight(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    if (!getFloat(obj, "height", c.height)) {
        setError(result, "missing or invalid height field");
        return false;
    }
    return requireRangeFloat(result, "height", c.height, 0.0f, 100.0f);
}

bool decodePanTilt(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    if (!getFloat(obj, "pan", c.pan) || !getFloat(obj, "tilt", c.tilt)) {
        setError(result, "missing or invalid pan_tilt field");
        return false;
    }
    return requireRangeFloat(result, "pan", c.pan, -90.0f, 90.0f) &&
           requireRangeFloat(result, "tilt", c.tilt, -90.0f, 90.0f);
}

bool decodeFace(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    if (!getString(obj, "mood", c.mood, sizeof(c.mood))) {
        setError(result, "missing or invalid mood field");
        return false;
    }
    static const char* kMoods[] = {"neutral", "happy", "angry", "surprised", "sleepy"};
    for (const char* m : kMoods) {
        if (std::strcmp(c.mood, m) == 0) return true;
    }
    setError(result, "unknown mood");
    return false;
}

bool decodeCalibrate(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    long servoIndex, offsetUs, sign = 1;
    if (!getInt(obj, "servo_index", servoIndex) || !getInt(obj, "offset_us", offsetUs)) {
        setError(result, "missing or invalid calibrate field");
        return false;
    }
    if (!obj["sign"].isNull() && !getInt(obj, "sign", sign)) {
        setError(result, "invalid sign field");
        return false;
    }
    if (!requireRangeInt(result, "servo_index", servoIndex, 0, SERVO_COUNT - 1)) return false;
    if (!requireRangeInt(result, "offset_us", offsetUs, -CALIBRATION_OFFSET_LIMIT_US, CALIBRATION_OFFSET_LIMIT_US)) return false;
    if (sign != -1 && sign != 1) {
        setError(result, "sign must be -1 or 1");
        return false;
    }
    c.servoIndex = static_cast<uint8_t>(servoIndex);
    c.offsetUs = static_cast<int16_t>(offsetUs);
    c.sign = static_cast<int8_t>(sign);
    return true;
}

bool decodeArmed(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    if (!getBool(obj, "armed", c.armed)) {
        setError(result, "missing or invalid armed field");
        return false;
    }
    return true;
}

bool decodeWriteOffsets(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    JsonVariantConst offsets = obj["offsets"];
    if (!offsets.is<JsonArrayConst>()) {
        setError(result, "offsets must be a list");
        return false;
    }
    JsonArrayConst arr = offsets.as<JsonArrayConst>();
    if (arr.size() == 0) {
        setError(result, "write_offsets requires at least one entry");
        return false;
    }
    if (arr.size() > SERVO_COUNT) {
        setError(result, "too many offset entries");
        return false;
    }
    bool seen[SERVO_COUNT] = {false};
    uint8_t i = 0;
    for (JsonVariantConst entry : arr) {
        long servoIndex, offsetUs, sign = 1;
        if (!getInt(entry, "servo_index", servoIndex) || !getInt(entry, "offset_us", offsetUs)) {
            setError(result, "invalid offset entry");
            return false;
        }
        if (!entry["sign"].isNull() && !getInt(entry, "sign", sign)) {
            setError(result, "invalid sign in offset entry");
            return false;
        }
        if (!requireRangeInt(result, "servo_index", servoIndex, 0, SERVO_COUNT - 1)) return false;
        if (!requireRangeInt(result, "offset_us", offsetUs, -CALIBRATION_OFFSET_LIMIT_US, CALIBRATION_OFFSET_LIMIT_US)) return false;
        if (sign != -1 && sign != 1) {
            setError(result, "sign must be -1 or 1");
            return false;
        }
        if (seen[servoIndex]) {
            setError(result, "duplicate servo_index in offsets");
            return false;
        }
        seen[servoIndex] = true;
        c.offsetEntries[i].servoIndex = static_cast<uint8_t>(servoIndex);
        c.offsetEntries[i].offsetUs = static_cast<int16_t>(offsetUs);
        c.offsetEntries[i].sign = static_cast<int8_t>(sign);
        ++i;
    }
    c.offsetEntryCount = i;
    return true;
}

bool decodeBenchPulse(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    long board, channel, pulseUs;
    if (!getInt(obj, "board", board) || !getInt(obj, "channel", channel) || !getInt(obj, "pulse_us", pulseUs)) {
        setError(result, "missing or invalid bench_pulse field");
        return false;
    }
    if (board != PCA9685_ADDR_BOARD_A && board != PCA9685_ADDR_BOARD_B) {
        setError(result, "unknown board address");
        return false;
    }
    if (!requireRangeInt(result, "channel", channel, 0, PCA9685_CHANNELS_PER_BOARD - 1)) return false;
    if (!requireRangeInt(result, "pulse_us", pulseUs, BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US)) return false;

    c.board = static_cast<uint8_t>(board);
    c.channel = static_cast<uint8_t>(channel);
    c.pulseUs = static_cast<uint16_t>(pulseUs);

    if (!obj["servo_index"].isNull()) {
        long servoIndex;
        if (!getInt(obj, "servo_index", servoIndex)) {
            setError(result, "invalid servo_index field");
            return false;
        }
        if (!requireRangeInt(result, "servo_index", servoIndex, 0, SERVO_COUNT - 1)) return false;
        c.servoIndex = static_cast<uint8_t>(servoIndex);
        c.hasServoIndex = true;
    }
    return true;
}

bool decodeRecordLimit(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    long servoIndex, pulseUs;
    char bound[8] = {0};
    if (!getInt(obj, "servo_index", servoIndex) || !getInt(obj, "pulse_us", pulseUs) ||
        !getString(obj, "bound", bound, sizeof(bound))) {
        setError(result, "missing or invalid record_limit field");
        return false;
    }
    if (!requireRangeInt(result, "servo_index", servoIndex, 0, SERVO_COUNT - 1)) return false;
    if (!requireRangeInt(result, "pulse_us", pulseUs, BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US)) return false;
    if (std::strcmp(bound, "min") == 0) {
        c.boundIsMax = false;
    } else if (std::strcmp(bound, "max") == 0) {
        c.boundIsMax = true;
    } else {
        setError(result, "unknown bound");
        return false;
    }
    c.servoIndex = static_cast<uint8_t>(servoIndex);
    c.pulseUs = static_cast<uint16_t>(pulseUs);
    return true;
}

bool decodeBenchHealthNote(JsonVariantConst obj, DecodeResult& result) {
    Command& c = result.command;
    long servoIndex;
    if (!getInt(obj, "servo_index", servoIndex) || !getString(obj, "note", c.note, sizeof(c.note))) {
        setError(result, "missing or invalid bench_health_note field");
        return false;
    }
    if (!requireRangeInt(result, "servo_index", servoIndex, 0, SERVO_COUNT - 1)) return false;
    c.servoIndex = static_cast<uint8_t>(servoIndex);
    return true;
}

}  // namespace

DecodeResult decodeCommand(const uint8_t* data, size_t len) {
    DecodeResult result;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (err) {
        setError(result, "malformed JSON");
        return result;
    }
    if (!doc.is<JsonObjectConst>()) {
        setError(result, "payload must be an object");
        return result;
    }
    JsonObjectConst obj = doc.as<JsonObjectConst>();

    long version;
    if (!getInt(obj, "v", version) || version != HX_PROTOCOL_VERSION) {
        setError(result, "unsupported protocol version");
        return result;
    }

    long seq;
    if (!getInt(obj, "seq", seq) || seq < 0 || static_cast<unsigned long>(seq) >= HX_SEQUENCE_MODULUS) {
        setError(result, "invalid sequence number");
        return result;
    }
    result.command.seq = static_cast<uint32_t>(seq);

    JsonVariantConst typeField = obj["type"];
    if (!typeField.is<const char*>()) {
        setError(result, "missing type field");
        return result;
    }
    const char* type = typeField.as<const char*>();

    bool decoded = false;
    if (std::strcmp(type, "walk") == 0) {
        result.command.type = CommandType::Walk;
        decoded = decodeWalk(obj, result);
    } else if (std::strcmp(type, "turn") == 0) {
        result.command.type = CommandType::Turn;
        decoded = decodeTurn(obj, result);
    } else if (std::strcmp(type, "stop") == 0) {
        result.command.type = CommandType::Stop;
        decoded = true;
    } else if (std::strcmp(type, "body_height") == 0) {
        result.command.type = CommandType::BodyHeight;
        decoded = decodeBodyHeight(obj, result);
    } else if (std::strcmp(type, "pan_tilt") == 0) {
        result.command.type = CommandType::PanTilt;
        decoded = decodePanTilt(obj, result);
    } else if (std::strcmp(type, "face") == 0) {
        result.command.type = CommandType::Face;
        decoded = decodeFace(obj, result);
    } else if (std::strcmp(type, "calibrate") == 0) {
        result.command.type = CommandType::Calibrate;
        decoded = decodeCalibrate(obj, result);
    } else if (std::strcmp(type, "calibration_mode") == 0) {
        result.command.type = CommandType::CalibrationMode;
        decoded = decodeArmed(obj, result);
    } else if (std::strcmp(type, "read_offsets") == 0) {
        result.command.type = CommandType::ReadOffsets;
        decoded = true;
    } else if (std::strcmp(type, "write_offsets") == 0) {
        result.command.type = CommandType::WriteOffsets;
        decoded = decodeWriteOffsets(obj, result);
    } else if (std::strcmp(type, "bench_mode") == 0) {
        result.command.type = CommandType::BenchMode;
        decoded = decodeArmed(obj, result);
    } else if (std::strcmp(type, "bench_pulse") == 0) {
        result.command.type = CommandType::BenchPulse;
        decoded = decodeBenchPulse(obj, result);
    } else if (std::strcmp(type, "record_limit") == 0) {
        result.command.type = CommandType::RecordLimit;
        decoded = decodeRecordLimit(obj, result);
    } else if (std::strcmp(type, "bench_health_note") == 0) {
        result.command.type = CommandType::BenchHealthNote;
        decoded = decodeBenchHealthNote(obj, result);
    } else if (std::strcmp(type, "ping") == 0) {
        result.command.type = CommandType::Ping;
        decoded = true;
    } else {
        setError(result, "unknown command type");
        return result;
    }

    result.ok = decoded;
    return result;
}

// --- Telemetry encode ------------------------------------------------------

namespace {

const char* motionTypeName(CommandType type) {
    switch (type) {
        case CommandType::Walk: return "walk";
        case CommandType::Turn: return "turn";
        case CommandType::Stop: return "stop";
        case CommandType::BodyHeight: return "body_height";
        case CommandType::PanTilt: return "pan_tilt";
        case CommandType::Face: return "face";
        default: return "stop";
    }
}

void writeMotionState(JsonObject dst, const MotionState& m) {
    dst["type"] = motionTypeName(m.type);
    switch (m.type) {
        case CommandType::Walk:
            dst["vx"] = m.vx;
            dst["vy"] = m.vy;
            dst["speed"] = m.speed;
            break;
        case CommandType::Turn:
            dst["rate"] = m.rate;
            dst["speed"] = m.speed;
            break;
        case CommandType::BodyHeight:
            dst["height"] = m.height;
            break;
        case CommandType::PanTilt:
            dst["pan"] = m.pan;
            dst["tilt"] = m.tilt;
            break;
        case CommandType::Face:
            dst["mood"] = m.mood;
            break;
        default:
            break;  // Stop carries no fields
    }
}

}  // namespace

size_t encodeTelemetry(const Telemetry& t, uint8_t* outBuf, size_t outBufLen) {
    JsonDocument doc;
    doc["v"] = HX_PROTOCOL_VERSION;
    doc["seq_echo"] = t.seqEcho;
    doc["ok"] = t.ok;
    if (t.error[0] != '\0') {
        doc["error"] = t.error;
    } else {
        doc["error"] = nullptr;
    }
    doc["fault_flags"] = t.faultFlags;
    if (t.hasGaitPhase) {
        doc["gait_phase"] = t.gaitPhase;
    } else {
        doc["gait_phase"] = nullptr;
    }
    if (t.hasRailMv) {
        doc["rail_mv"] = t.railMv;
    } else {
        doc["rail_mv"] = nullptr;
    }
    doc["link_timeout_s"] = t.linkTimeoutS;
    doc["calibration_armed"] = t.calibrationArmed;
    doc["bench_armed"] = t.benchArmed;
    doc["robot_assembled"] = t.robotAssembled;
    doc["ik_clip_count"] = t.ikClipCount;
    doc["ik_clip_worst_mm"] = t.ikClipWorstMm;
    doc["joint_clip_count"] = t.jointClipCount;
    doc["joint_clip_worst_deg"] = t.jointClipWorstDeg;

    if (t.hasLastApplied) {
        JsonObject la = doc["last_applied"].to<JsonObject>();
        writeMotionState(la, t.lastApplied);
    } else {
        doc["last_applied"] = nullptr;
    }

    if (t.hasProfiles) {
        JsonArray arr = doc["profiles"].to<JsonArray>();
        for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
            const ServoProfile& p = t.profiles[i];
            JsonObject entry = arr.add<JsonObject>();
            entry["servo_index"] = p.servoIndex;
            entry["offset_us"] = p.offsetUs;
            entry["sign"] = p.sign;
            if (p.hasMinDeg) {
                entry["min_deg_from_neutral"] = p.minDegFromNeutral;
            } else {
                entry["min_deg_from_neutral"] = nullptr;
            }
            if (p.hasMaxDeg) {
                entry["max_deg_from_neutral"] = p.maxDegFromNeutral;
            } else {
                entry["max_deg_from_neutral"] = nullptr;
            }
            entry["note"] = p.note;
        }
    } else {
        doc["profiles"] = nullptr;
    }

    size_t written = serializeJson(doc, reinterpret_cast<char*>(outBuf), outBufLen);
    if (written == 0 || written >= outBufLen) {
        return 0;
    }
    return written;
}
