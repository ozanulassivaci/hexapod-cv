#include <unity.h>

#include <ArduinoJson.h>
#include <cstring>

#include "Protocol.h"

void setUp(void) {}
void tearDown(void) {}

static DecodeResult decodeStr(const char* json) {
    return decodeCommand(reinterpret_cast<const uint8_t*>(json), std::strlen(json));
}

// --- Round-trips (decode) ------------------------------------------------

void test_decode_walk(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":5,"type":"walk","vx":0.5,"vy":-0.3,"speed":40})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(5, r.command.seq);
    TEST_ASSERT_TRUE(CommandType::Walk == r.command.type);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, r.command.vx);
    TEST_ASSERT_EQUAL_FLOAT(-0.3f, r.command.vy);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, r.command.speed);
}

void test_decode_turn(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"turn","rate":-1.0,"speed":0})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(CommandType::Turn == r.command.type);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, r.command.rate);
}

void test_decode_stop(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"stop"})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(CommandType::Stop == r.command.type);
}

void test_decode_body_height(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"body_height","height":75})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL_FLOAT(75.0f, r.command.height);
}

void test_decode_pan_tilt(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"pan_tilt","pan":-45,"tilt":30})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL_FLOAT(-45.0f, r.command.pan);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, r.command.tilt);
}

void test_decode_face(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"face","mood":"happy"})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL_STRING("happy", r.command.mood);
}

void test_decode_face_rejects_unknown_mood(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"face","mood":"furious"})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_calibrate(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"calibrate","servo_index":17,"offset_us":-500,"sign":-1})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(17, r.command.servoIndex);
    TEST_ASSERT_EQUAL(-500, r.command.offsetUs);
    TEST_ASSERT_EQUAL(-1, r.command.sign);
}

void test_decode_calibrate_sign_defaults_to_one(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"calibrate","servo_index":0,"offset_us":0})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(1, r.command.sign);
}

void test_decode_calibration_mode(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"calibration_mode","armed":true})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(r.command.armed);
}

void test_decode_read_offsets(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"read_offsets"})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(CommandType::ReadOffsets == r.command.type);
}

void test_decode_write_offsets(void) {
    DecodeResult r = decodeStr(
        R"({"v":1,"seq":1,"type":"write_offsets","offsets":)"
        R"([{"servo_index":0,"offset_us":10,"sign":1},{"servo_index":5,"offset_us":-20,"sign":-1}]})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(2, r.command.offsetEntryCount);
    TEST_ASSERT_EQUAL(0, r.command.offsetEntries[0].servoIndex);
    TEST_ASSERT_EQUAL(10, r.command.offsetEntries[0].offsetUs);
    TEST_ASSERT_EQUAL(5, r.command.offsetEntries[1].servoIndex);
    TEST_ASSERT_EQUAL(-1, r.command.offsetEntries[1].sign);
}

void test_decode_write_offsets_rejects_duplicate_index(void) {
    DecodeResult r = decodeStr(
        R"({"v":1,"seq":1,"type":"write_offsets","offsets":)"
        R"([{"servo_index":0,"offset_us":1},{"servo_index":0,"offset_us":2}]})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_write_offsets_rejects_empty(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"write_offsets","offsets":[]})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_bench_mode(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"bench_mode","armed":false})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_FALSE(r.command.armed);
}

void test_decode_bench_pulse_without_servo_index(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"bench_pulse","board":64,"channel":0,"pulse_us":1500})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(64, r.command.board);
    TEST_ASSERT_EQUAL(0, r.command.channel);
    TEST_ASSERT_EQUAL(1500, r.command.pulseUs);
    TEST_ASSERT_FALSE(r.command.hasServoIndex);
}

void test_decode_bench_pulse_with_servo_index(void) {
    DecodeResult r = decodeStr(
        R"({"v":1,"seq":1,"type":"bench_pulse","board":65,"channel":15,"pulse_us":2000,"servo_index":3})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(65, r.command.board);
    TEST_ASSERT_TRUE(r.command.hasServoIndex);
    TEST_ASSERT_EQUAL(3, r.command.servoIndex);
}

void test_decode_bench_pulse_rejects_unknown_board(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"bench_pulse","board":66,"channel":0,"pulse_us":1500})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_bench_release(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"bench_release","board":64,"channel":7})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(64, r.command.board);
    TEST_ASSERT_EQUAL(7, r.command.channel);
}

void test_decode_bench_release_rejects_unknown_board(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"bench_release","board":66,"channel":0})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_bench_release_rejects_out_of_range_channel(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"bench_release","board":64,"channel":16})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_record_limit_min(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"record_limit","servo_index":3,"bound":"min","pulse_us":950})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(3, r.command.servoIndex);
    TEST_ASSERT_FALSE(r.command.boundIsMax);
    TEST_ASSERT_EQUAL(950, r.command.pulseUs);
}

void test_decode_record_limit_max(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"record_limit","servo_index":3,"bound":"max","pulse_us":2050})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(r.command.boundIsMax);
}

void test_decode_record_limit_rejects_unknown_bound(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"record_limit","servo_index":3,"bound":"sideways","pulse_us":1500})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_bench_health_note(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"bench_health_note","servo_index":9,"note":"buzzes"})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL(9, r.command.servoIndex);
    TEST_ASSERT_EQUAL_STRING("buzzes", r.command.note);
}

void test_decode_ping(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"ping"})");
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(CommandType::Ping == r.command.type);
}

// --- Range validation ------------------------------------------------------

void test_decode_rejects_out_of_range_walk(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"walk","vx":1.5,"vy":0,"speed":0})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_out_of_range_speed(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"walk","vx":0,"vy":0,"speed":101})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_accepts_boundary_values(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"walk","vx":-1.0,"vy":1.0,"speed":100})");
    TEST_ASSERT_TRUE(r.ok);
}

void test_decode_rejects_bool_where_number_expected(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"walk","vx":true,"vy":0,"speed":0})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_out_of_range_servo_index(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"calibrate","servo_index":18,"offset_us":0})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_bad_sign(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"calibrate","servo_index":0,"offset_us":0,"sign":2})");
    TEST_ASSERT_FALSE(r.ok);
}

// --- Malformed input ---------------------------------------------------

void test_decode_rejects_malformed_json(void) {
    DecodeResult r = decodeStr("not json{{{");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_non_object(void) {
    DecodeResult r = decodeStr("[1,2,3]");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_wrong_version(void) {
    DecodeResult r = decodeStr(R"({"v":2,"seq":1,"type":"stop"})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_missing_version(void) {
    DecodeResult r = decodeStr(R"({"seq":1,"type":"stop"})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_unknown_type(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"dance"})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_missing_field(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"walk","vx":0.1})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_negative_seq(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":-1,"type":"stop"})");
    TEST_ASSERT_FALSE(r.ok);
}

void test_decode_rejects_bool_as_armed(void) {
    DecodeResult r = decodeStr(R"({"v":1,"seq":1,"type":"calibration_mode","armed":1})");
    TEST_ASSERT_FALSE(r.ok);
}

// --- Telemetry encode ----------------------------------------------------

void test_encode_telemetry_minimal_shape(void) {
    Telemetry t;
    t.seqEcho = 42;
    t.ok = true;
    t.faultFlags = 0;
    t.linkTimeoutS = 1.0f;
    t.calibrationArmed = false;
    t.benchArmed = true;
    t.robotAssembled = false;

    uint8_t buf[512];
    size_t len = encodeTelemetry(t, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, len);

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf, len);
    TEST_ASSERT_FALSE(err);
    TEST_ASSERT_EQUAL(1, doc["v"].as<int>());
    TEST_ASSERT_EQUAL(42, doc["seq_echo"].as<unsigned long>());
    TEST_ASSERT_TRUE(doc["ok"].as<bool>());
    TEST_ASSERT_TRUE(doc["error"].isNull());
    TEST_ASSERT_TRUE(doc["gait_phase"].isNull());
    TEST_ASSERT_TRUE(doc["rail_mv"].isNull());
    TEST_ASSERT_TRUE(doc["last_applied"].isNull());
    TEST_ASSERT_TRUE(doc["profiles"].isNull());
    TEST_ASSERT_FALSE(doc["calibration_armed"].as<bool>());
    TEST_ASSERT_TRUE(doc["bench_armed"].as<bool>());
    TEST_ASSERT_FALSE(doc["robot_assembled"].as<bool>());
}

void test_encode_telemetry_robot_assembled_true(void) {
    Telemetry t;
    t.robotAssembled = true;

    uint8_t buf[512];
    size_t len = encodeTelemetry(t, buf, sizeof(buf));
    JsonDocument doc;
    deserializeJson(doc, buf, len);
    TEST_ASSERT_TRUE(doc["robot_assembled"].as<bool>());
}

void test_encode_telemetry_default_clip_stats_are_zero(void) {
    Telemetry t;

    uint8_t buf[512];
    size_t len = encodeTelemetry(t, buf, sizeof(buf));
    JsonDocument doc;
    deserializeJson(doc, buf, len);
    TEST_ASSERT_EQUAL(0, doc["ik_clip_count"].as<unsigned long>());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, doc["ik_clip_worst_mm"].as<float>());
    TEST_ASSERT_EQUAL(0, doc["joint_clip_count"].as<unsigned long>());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, doc["joint_clip_worst_deg"].as<float>());
}

void test_encode_telemetry_clip_stats(void) {
    Telemetry t;
    t.ikClipCount = 3;
    t.ikClipWorstMm = 12.5f;
    t.jointClipCount = 1;
    t.jointClipWorstDeg = 2.25f;

    uint8_t buf[512];
    size_t len = encodeTelemetry(t, buf, sizeof(buf));
    JsonDocument doc;
    deserializeJson(doc, buf, len);
    TEST_ASSERT_EQUAL(3, doc["ik_clip_count"].as<unsigned long>());
    TEST_ASSERT_EQUAL_FLOAT(12.5f, doc["ik_clip_worst_mm"].as<float>());
    TEST_ASSERT_EQUAL(1, doc["joint_clip_count"].as<unsigned long>());
    TEST_ASSERT_EQUAL_FLOAT(2.25f, doc["joint_clip_worst_deg"].as<float>());
}

void test_encode_telemetry_with_error(void) {
    Telemetry t;
    t.ok = false;
    std::strncpy(t.error, "bench mode not armed", sizeof(t.error) - 1);

    uint8_t buf[512];
    size_t len = encodeTelemetry(t, buf, sizeof(buf));
    JsonDocument doc;
    deserializeJson(doc, buf, len);
    TEST_ASSERT_FALSE(doc["ok"].as<bool>());
    TEST_ASSERT_EQUAL_STRING("bench mode not armed", doc["error"].as<const char*>());
}

void test_encode_telemetry_with_last_applied_walk(void) {
    Telemetry t;
    t.hasLastApplied = true;
    t.lastApplied.type = CommandType::Walk;
    t.lastApplied.vx = 0.5f;
    t.lastApplied.vy = -0.25f;
    t.lastApplied.speed = 30.0f;

    uint8_t buf[512];
    size_t len = encodeTelemetry(t, buf, sizeof(buf));
    JsonDocument doc;
    deserializeJson(doc, buf, len);
    TEST_ASSERT_EQUAL_STRING("walk", doc["last_applied"]["type"].as<const char*>());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, doc["last_applied"]["vx"].as<float>());
    TEST_ASSERT_EQUAL_FLOAT(30.0f, doc["last_applied"]["speed"].as<float>());
}

void test_encode_telemetry_with_profiles(void) {
    Telemetry t;
    t.hasProfiles = true;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        t.profiles[i] = ServoProfile::defaultFor(i);
    }
    t.profiles[3].offsetUs = 20;
    t.profiles[3].sign = -1;
    t.profiles[3].hasMinDeg = true;
    t.profiles[3].minDegFromNeutral = -49.5f;
    std::strncpy(t.profiles[3].note, "buzzes", sizeof(t.profiles[3].note) - 1);

    uint8_t buf[4096];
    size_t len = encodeTelemetry(t, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, len);

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf, len);
    TEST_ASSERT_FALSE(err);
    JsonArrayConst profiles = doc["profiles"].as<JsonArrayConst>();
    TEST_ASSERT_EQUAL(SERVO_COUNT, profiles.size());
    JsonObjectConst p3 = profiles[3];
    TEST_ASSERT_EQUAL(20, p3["offset_us"].as<int>());
    TEST_ASSERT_EQUAL(-1, p3["sign"].as<int>());
    TEST_ASSERT_EQUAL_FLOAT(-49.5f, p3["min_deg_from_neutral"].as<float>());
    TEST_ASSERT_TRUE(p3["max_deg_from_neutral"].isNull());
    TEST_ASSERT_EQUAL_STRING("buzzes", p3["note"].as<const char*>());

    JsonObjectConst p0 = profiles[0];
    TEST_ASSERT_TRUE(p0["min_deg_from_neutral"].isNull());
    TEST_ASSERT_TRUE(p0["max_deg_from_neutral"].isNull());
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_walk);
    RUN_TEST(test_decode_turn);
    RUN_TEST(test_decode_stop);
    RUN_TEST(test_decode_body_height);
    RUN_TEST(test_decode_pan_tilt);
    RUN_TEST(test_decode_face);
    RUN_TEST(test_decode_face_rejects_unknown_mood);
    RUN_TEST(test_decode_calibrate);
    RUN_TEST(test_decode_calibrate_sign_defaults_to_one);
    RUN_TEST(test_decode_calibration_mode);
    RUN_TEST(test_decode_read_offsets);
    RUN_TEST(test_decode_write_offsets);
    RUN_TEST(test_decode_write_offsets_rejects_duplicate_index);
    RUN_TEST(test_decode_write_offsets_rejects_empty);
    RUN_TEST(test_decode_bench_mode);
    RUN_TEST(test_decode_bench_pulse_without_servo_index);
    RUN_TEST(test_decode_bench_pulse_with_servo_index);
    RUN_TEST(test_decode_bench_pulse_rejects_unknown_board);
    RUN_TEST(test_decode_bench_release);
    RUN_TEST(test_decode_bench_release_rejects_unknown_board);
    RUN_TEST(test_decode_bench_release_rejects_out_of_range_channel);
    RUN_TEST(test_decode_record_limit_min);
    RUN_TEST(test_decode_record_limit_max);
    RUN_TEST(test_decode_record_limit_rejects_unknown_bound);
    RUN_TEST(test_decode_bench_health_note);
    RUN_TEST(test_decode_ping);
    RUN_TEST(test_decode_rejects_out_of_range_walk);
    RUN_TEST(test_decode_rejects_out_of_range_speed);
    RUN_TEST(test_decode_accepts_boundary_values);
    RUN_TEST(test_decode_rejects_bool_where_number_expected);
    RUN_TEST(test_decode_rejects_out_of_range_servo_index);
    RUN_TEST(test_decode_rejects_bad_sign);
    RUN_TEST(test_decode_rejects_malformed_json);
    RUN_TEST(test_decode_rejects_non_object);
    RUN_TEST(test_decode_rejects_wrong_version);
    RUN_TEST(test_decode_rejects_missing_version);
    RUN_TEST(test_decode_rejects_unknown_type);
    RUN_TEST(test_decode_rejects_missing_field);
    RUN_TEST(test_decode_rejects_negative_seq);
    RUN_TEST(test_decode_rejects_bool_as_armed);
    RUN_TEST(test_encode_telemetry_minimal_shape);
    RUN_TEST(test_encode_telemetry_robot_assembled_true);
    RUN_TEST(test_encode_telemetry_default_clip_stats_are_zero);
    RUN_TEST(test_encode_telemetry_clip_stats);
    RUN_TEST(test_encode_telemetry_with_error);
    RUN_TEST(test_encode_telemetry_with_last_applied_walk);
    RUN_TEST(test_encode_telemetry_with_profiles);
    return UNITY_END();
}
