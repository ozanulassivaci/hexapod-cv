import pytest

from transport.generated_constants import PROTOCOL_VERSION, SEQUENCE_MODULUS
from transport.protocol import (
    BodyHeightCommand,
    CalibrateCommand,
    CalibrationModeCommand,
    FaceCommand,
    Mood,
    PanTiltCommand,
    PingCommand,
    ProtocolError,
    ReadOffsetsCommand,
    ServoOffset,
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
    WriteOffsetsCommand(offsets=[ServoOffset(0, 10, 1), ServoOffset(5, -20, -1)]),
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
        last_applied=WalkCommand(vx=0.1, vy=0.2, speed=10.0),
        offsets=(ServoOffset(0, 5, 1), ServoOffset(1, -5, -1)),
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
        last_applied=None,
        offsets=None,
    )
    decoded = decode_telemetry(encode_telemetry(telemetry))
    assert decoded == telemetry


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
        lambda: ServoOffset(servo_index=18, offset_us=0),
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
        WriteOffsetsCommand(offsets=[ServoOffset(0, 1, 1), ServoOffset(0, 2, 1)])


def test_write_offsets_rejects_too_many_entries():
    with pytest.raises(ProtocolError):
        WriteOffsetsCommand(offsets=[ServoOffset(i, 0, 1) for i in range(18)] + [ServoOffset(0, 1, 1)])


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


def test_decode_telemetry_rejects_missing_field():
    payload = f'{{"v":{PROTOCOL_VERSION},"seq_echo":0,"ok":true}}'.encode()
    with pytest.raises(ProtocolError):
        decode_telemetry(payload)


def test_decode_telemetry_rejects_out_of_range_gait_phase():
    payload = (
        f'{{"v":{PROTOCOL_VERSION},"seq_echo":0,"ok":true,"fault_flags":0,'
        f'"link_timeout_s":1.0,"calibration_armed":false,"gait_phase":1.5}}'
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
