import json

import pytest

from transport.generated_constants import PROTOCOL_VERSION, SEQUENCE_MODULUS
from transport.protocol import (
    BENCH_PULSE_MAX_US,
    BENCH_PULSE_MIN_US,
    BenchHealthNoteCommand,
    BenchModeCommand,
    BenchPulseCommand,
    BodyHeightCommand,
    CalibrateCommand,
    CalibrationModeCommand,
    FaceCommand,
    FAULT_IK_CLIP,
    HEALTH_NOTE_MAX_LEN,
    LimitBound,
    Mood,
    PanTiltCommand,
    PingCommand,
    ProtocolError,
    ReadOffsetsCommand,
    RecordLimitCommand,
    ServoProfile,
    StopCommand,
    Telemetry,
    TurnCommand,
    WalkCommand,
    WriteOffsetsCommand,
    decode_command,
    decode_telemetry,
    encode_command,
    encode_telemetry,
    next_sequence,
    sequence_is_newer,
)

# --- Round-trips -------------------------------------------------------

ROUND_TRIP_COMMANDS = [
    WalkCommand(vx=0.5, vy=-0.3, speed=40.0),
    TurnCommand(rate=-1.0, speed=0.0),
    StopCommand(),
    BodyHeightCommand(height=75.0),
    PanTiltCommand(pan=-45.0, tilt=30.0),
    FaceCommand(mood=Mood.HAPPY),
    CalibrateCommand(servo_index=17, offset_us=-500, sign=-1),
    CalibrationModeCommand(armed=True),
    ReadOffsetsCommand(),
    WriteOffsetsCommand(offsets=[ServoProfile(0, 10, 1), ServoProfile(5, -20, -1)]),
    BenchModeCommand(armed=True),
    BenchPulseCommand(board=0x40, channel=0, pulse_us=1500),
    BenchPulseCommand(board=0x41, channel=15, pulse_us=BENCH_PULSE_MAX_US),
    BenchPulseCommand(board=0x40, channel=3, pulse_us=1600, servo_index=7),
    RecordLimitCommand(servo_index=3, bound=LimitBound.MIN, pulse_us=900),
    RecordLimitCommand(servo_index=3, bound=LimitBound.MAX, pulse_us=2100),
    BenchHealthNoteCommand(servo_index=9, note="buzzes at low end"),
    PingCommand(),
]


@pytest.mark.parametrize("command", ROUND_TRIP_COMMANDS, ids=lambda c: type(c).__name__)
def test_command_round_trip(command):
    data = encode_command(command, seq=123)
    received = decode_command(data)
    assert received.seq == 123
    assert received.command == command


def test_telemetry_round_trip_full():
    telemetry = Telemetry(
        seq_echo=42,
        ok=False,
        error="calibration not armed",
        fault_flags=3,
        gait_phase=0.75,
        rail_mv=5950,
        link_timeout_s=1.0,
        calibration_armed=True,
        bench_armed=False,
        robot_assembled=True,
        last_applied=WalkCommand(vx=0.1, vy=0.2, speed=10.0),
        profiles=(
            ServoProfile(0, 5, 1, -40.0, 35.0, "buzzes at low end"),
            ServoProfile(1, -5, -1),
        ),
    )
    decoded = decode_telemetry(encode_telemetry(telemetry))
    assert decoded == telemetry


def test_telemetry_round_trip_minimal():
    telemetry = Telemetry(
        seq_echo=0,
        ok=True,
        error=None,
        fault_flags=0,
        gait_phase=None,
        rail_mv=None,
        link_timeout_s=1.0,
        calibration_armed=False,
        bench_armed=False,
        robot_assembled=False,
        last_applied=None,
        profiles=None,
    )
    decoded = decode_telemetry(encode_telemetry(telemetry))
    assert decoded == telemetry


def test_telemetry_round_trip_carries_clip_stats():
    telemetry = Telemetry(
        seq_echo=1,
        ok=True,
        error=None,
        fault_flags=FAULT_IK_CLIP,
        gait_phase=0.5,
        rail_mv=None,
        link_timeout_s=1.0,
        calibration_armed=False,
        bench_armed=False,
        robot_assembled=False,
        last_applied=None,
        profiles=None,
        ik_clip_count=3,
        ik_clip_worst_mm=12.5,
        joint_clip_count=1,
        joint_clip_worst_deg=2.25,
    )
    decoded = decode_telemetry(encode_telemetry(telemetry))
    assert decoded == telemetry


def test_telemetry_clip_fields_default_to_zero():
    telemetry = Telemetry(
        seq_echo=0,
        ok=True,
        error=None,
        fault_flags=0,
        gait_phase=None,
        rail_mv=None,
        link_timeout_s=1.0,
        calibration_armed=False,
        bench_armed=False,
        robot_assembled=False,
        last_applied=None,
        profiles=None,
    )
    assert telemetry.ik_clip_count == 0
    assert telemetry.ik_clip_worst_mm == 0.0
    assert telemetry.joint_clip_count == 0
    assert telemetry.joint_clip_worst_deg == 0.0


def test_telemetry_decode_rejects_missing_ik_clip_count():
    telemetry = Telemetry(
        seq_echo=0,
        ok=True,
        error=None,
        fault_flags=0,
        gait_phase=None,
        rail_mv=None,
        link_timeout_s=1.0,
        calibration_armed=False,
        bench_armed=False,
        robot_assembled=False,
        last_applied=None,
        profiles=None,
    )
    encoded = encode_telemetry(telemetry)
    payload = json.loads(encoded)
    del payload["ik_clip_count"]
    with pytest.raises(ProtocolError):
        decode_telemetry(json.dumps(payload).encode("utf-8"))


def test_telemetry_decode_rejects_negative_joint_clip_worst_deg():
    telemetry = Telemetry(
        seq_echo=0,
        ok=True,
        error=None,
        fault_flags=0,
        gait_phase=None,
        rail_mv=None,
        link_timeout_s=1.0,
        calibration_armed=False,
        bench_armed=False,
        robot_assembled=False,
        last_applied=None,
        profiles=None,
    )
    encoded = encode_telemetry(telemetry)
    payload = json.loads(encoded)
    payload["joint_clip_worst_deg"] = -1.0
    with pytest.raises(ProtocolError):
        decode_telemetry(json.dumps(payload).encode("utf-8"))


def test_telemetry_decode_rejects_missing_robot_assembled():
    telemetry = Telemetry(
        seq_echo=0,
        ok=True,
        error=None,
        fault_flags=0,
        gait_phase=None,
        rail_mv=None,
        link_timeout_s=1.0,
        calibration_armed=False,
        bench_armed=False,
        robot_assembled=False,
        last_applied=None,
        profiles=None,
    )
    encoded = encode_telemetry(telemetry)
    payload = json.loads(encoded)
    del payload["robot_assembled"]
    with pytest.raises(ProtocolError):
        decode_telemetry(json.dumps(payload).encode("utf-8"))


# --- write_offsets stays scoped to offset/sign, never limits/notes --------


def test_write_offsets_wire_fields_exclude_limits_and_note():
    profile = ServoProfile(0, offset_us=10, sign=-1, min_deg_from_neutral=-20.0, max_deg_from_neutral=30.0, note="hi")
    command = WriteOffsetsCommand(offsets=[profile])
    fields = command._wire_fields()
    assert fields == {"offsets": [{"servo_index": 0, "offset_us": 10, "sign": -1}]}


# --- Range validation ----------------------------------------------------


@pytest.mark.parametrize(
    "factory",
    [
        lambda: WalkCommand(vx=1.01, vy=0, speed=0),
        lambda: WalkCommand(vx=0, vy=-1.01, speed=0),
        lambda: WalkCommand(vx=0, vy=0, speed=100.01),
        lambda: WalkCommand(vx=0, vy=0, speed=-0.01),
        lambda: TurnCommand(rate=1.5, speed=0),
        lambda: BodyHeightCommand(height=100.5),
        lambda: BodyHeightCommand(height=-1),
        lambda: PanTiltCommand(pan=91, tilt=0),
        lambda: PanTiltCommand(pan=0, tilt=-91),
        lambda: CalibrateCommand(servo_index=18, offset_us=0),
        lambda: CalibrateCommand(servo_index=-1, offset_us=0),
        lambda: CalibrateCommand(servo_index=0, offset_us=501),
        lambda: CalibrateCommand(servo_index=0, offset_us=0, sign=0),
        lambda: ServoProfile(servo_index=18, offset_us=0),
        lambda: ServoProfile(servo_index=0, min_deg_from_neutral=10.0, max_deg_from_neutral=-10.0),
        lambda: ServoProfile(servo_index=0, min_deg_from_neutral=0.0, max_deg_from_neutral=0.0),
        lambda: ServoProfile(servo_index=0, min_deg_from_neutral=-91.0),
        lambda: ServoProfile(servo_index=0, max_deg_from_neutral=91.0),
        lambda: ServoProfile(servo_index=0, note="x" * (HEALTH_NOTE_MAX_LEN + 1)),
        lambda: BenchModeCommand(armed="yes"),
        lambda: BenchPulseCommand(board=0x42, channel=0, pulse_us=1500),
        lambda: BenchPulseCommand(board=0x40, channel=16, pulse_us=1500),
        lambda: BenchPulseCommand(board=0x40, channel=0, pulse_us=BENCH_PULSE_MIN_US - 1),
        lambda: BenchPulseCommand(board=0x40, channel=0, pulse_us=BENCH_PULSE_MAX_US + 1),
        lambda: BenchPulseCommand(board=0x40, channel=0, pulse_us=1500, servo_index=18),
        lambda: BenchPulseCommand(board=0x40, channel=0, pulse_us=1500, servo_index=-1),
        lambda: RecordLimitCommand(servo_index=18, bound=LimitBound.MIN, pulse_us=1500),
        lambda: RecordLimitCommand(servo_index=0, bound="min", pulse_us=1500),
        lambda: RecordLimitCommand(servo_index=0, bound=LimitBound.MIN, pulse_us=100),
        lambda: BenchHealthNoteCommand(servo_index=18, note="x"),
        lambda: BenchHealthNoteCommand(servo_index=0, note="x" * (HEALTH_NOTE_MAX_LEN + 1)),
    ],
)
def test_out_of_range_construction_rejected(factory):
    with pytest.raises(ProtocolError):
        factory()


def test_boundary_values_are_accepted():
    WalkCommand(vx=-1.0, vy=1.0, speed=100.0)
    WalkCommand(vx=1.0, vy=-1.0, speed=0.0)
    CalibrateCommand(servo_index=0, offset_us=-500, sign=-1)
    CalibrateCommand(servo_index=17, offset_us=500, sign=1)
    PanTiltCommand(pan=-90.0, tilt=90.0)
    # servo_index=0 is a coxa (bipolar, servo_index % 3 == 0), whose legal
    # degree-from-neutral range is exactly [-90, +90] -- the boundary
    # equivalent of the old BENCH_PULSE_MIN_US/MAX_US pulse boundaries.
    ServoProfile(servo_index=0, min_deg_from_neutral=-90.0, max_deg_from_neutral=90.0)
    BenchPulseCommand(board=0x40, channel=0, pulse_us=BENCH_PULSE_MIN_US)
    BenchPulseCommand(board=0x41, channel=15, pulse_us=BENCH_PULSE_MAX_US)


def test_servo_profile_limits_default_to_none_not_a_fake_value():
    profile = ServoProfile(servo_index=0)
    assert profile.min_deg_from_neutral is None
    assert profile.max_deg_from_neutral is None
    assert profile.note == ""


def test_walk_rejects_nan_and_infinity():
    with pytest.raises(ProtocolError):
        WalkCommand(vx=float("nan"), vy=0, speed=0)
    with pytest.raises(ProtocolError):
        WalkCommand(vx=float("inf"), vy=0, speed=0)


def test_bool_rejected_where_number_expected():
    with pytest.raises(ProtocolError):
        WalkCommand(vx=True, vy=0, speed=0)


def test_calibrate_sign_must_be_plus_or_minus_one():
    with pytest.raises(ProtocolError):
        CalibrateCommand(servo_index=0, offset_us=0, sign=2)


def test_write_offsets_rejects_empty():
    with pytest.raises(ProtocolError):
        WriteOffsetsCommand(offsets=[])


def test_write_offsets_rejects_duplicate_servo_index():
    with pytest.raises(ProtocolError):
        WriteOffsetsCommand(offsets=[ServoProfile(0, 1, 1), ServoProfile(0, 2, 1)])


def test_write_offsets_rejects_too_many_entries():
    with pytest.raises(ProtocolError):
        WriteOffsetsCommand(offsets=[ServoProfile(i, 0, 1) for i in range(18)] + [ServoProfile(0, 1, 1)])


def test_face_rejects_non_mood():
    with pytest.raises(ProtocolError):
        FaceCommand(mood="happy")  # must be the Mood enum, not a raw string


# --- Decode-side malformed input -----------------------------------------


def test_decode_rejects_malformed_json():
    with pytest.raises(ProtocolError):
        decode_command(b"not json{{{")


def test_decode_rejects_non_object_json():
    with pytest.raises(ProtocolError):
        decode_command(b"[1,2,3]")


def test_decode_rejects_wrong_version():
    payload = f'{{"v":{PROTOCOL_VERSION + 1},"seq":0,"type":"stop"}}'.encode()
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_decode_rejects_missing_version():
    payload = b'{"seq":0,"type":"stop"}'
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_decode_rejects_unknown_type():
    payload = f'{{"v":{PROTOCOL_VERSION},"seq":0,"type":"dance"}}'.encode()
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_decode_rejects_missing_field():
    payload = f'{{"v":{PROTOCOL_VERSION},"seq":0,"type":"walk","vx":0.1}}'.encode()
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_decode_rejects_out_of_range_field():
    payload = (
        f'{{"v":{PROTOCOL_VERSION},"seq":0,"type":"walk","vx":5,"vy":0,"speed":0}}'
    ).encode()
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_decode_rejects_negative_seq():
    payload = f'{{"v":{PROTOCOL_VERSION},"seq":-1,"type":"stop"}}'.encode()
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_decode_rejects_bool_as_armed():
    payload = f'{{"v":{PROTOCOL_VERSION},"seq":0,"type":"calibration_mode","armed":1}}'.encode()
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_decode_rejects_unknown_limit_bound():
    payload = (
        f'{{"v":{PROTOCOL_VERSION},"seq":0,"type":"record_limit",'
        f'"servo_index":0,"bound":"sideways","pulse_us":1500}}'
    ).encode()
    with pytest.raises(ProtocolError):
        decode_command(payload)


def test_decode_telemetry_rejects_missing_field():
    payload = f'{{"v":{PROTOCOL_VERSION},"seq_echo":0,"ok":true}}'.encode()
    with pytest.raises(ProtocolError):
        decode_telemetry(payload)


def test_decode_telemetry_rejects_missing_bench_armed():
    payload = (
        f'{{"v":{PROTOCOL_VERSION},"seq_echo":0,"ok":true,"fault_flags":0,'
        f'"link_timeout_s":1.0,"calibration_armed":false}}'
    ).encode()
    with pytest.raises(ProtocolError):
        decode_telemetry(payload)


def test_decode_telemetry_rejects_out_of_range_gait_phase():
    payload = (
        f'{{"v":{PROTOCOL_VERSION},"seq_echo":0,"ok":true,"fault_flags":0,'
        f'"link_timeout_s":1.0,"calibration_armed":false,"bench_armed":false,"gait_phase":1.5}}'
    ).encode()
    with pytest.raises(ProtocolError):
        decode_telemetry(payload)


# --- Sequence numbers ------------------------------------------------------


def test_sequence_is_newer_simple_increment():
    assert sequence_is_newer(1, 0) is True
    assert sequence_is_newer(0, 1) is False


def test_sequence_equal_is_not_newer():
    assert sequence_is_newer(5, 5) is False


def test_sequence_is_newer_handles_wraparound():
    # candidate has wrapped around past the modulus, reference is near the top
    reference = SEQUENCE_MODULUS - 1
    candidate = 0
    assert sequence_is_newer(candidate, reference) is True
    assert sequence_is_newer(reference, candidate) is False


def test_sequence_far_older_is_not_newer():
    reference = 1000
    candidate = 10
    assert sequence_is_newer(candidate, reference) is False


def test_next_sequence_wraps_at_modulus():
    assert next_sequence(SEQUENCE_MODULUS - 1) == 0
    assert next_sequence(0) == 1
