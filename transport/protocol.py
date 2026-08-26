"""Wire protocol for PC-to-robot commands and robot-to-PC telemetry.

See docs/protocol.md for the full design rationale. Summary: JSON over UDP,
intent-based commands (not streamed foot targets), a single monotonic
sequence counter per PC session, and "latest wins, stale is dropped" for
both commands and telemetry -- the same rule MJPEGStream already applies to
frames.

Every command is rejected loudly (ProtocolError) at construction time if
out of range, and at decode time if malformed -- this module never hands a
transport layer a value it hasn't already validated.
"""

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Callable, ClassVar

from transport.generated_constants import PROTOCOL_VERSION, SEQUENCE_MODULUS

# Command-field bounds. Not shared with firmware via codegen (unlike
# PROTOCOL_VERSION/SEQUENCE_BITS/timeouts) -- a mismatch here is a rejected
# command, not a misfired failsafe, so it doesn't carry the same risk.
SERVO_COUNT = 18
CALIBRATION_OFFSET_LIMIT_US = 500
PAN_TILT_MIN_DEG = -90.0
PAN_TILT_MAX_DEG = 90.0

FAULT_NONE = 0
FAULT_LINK_TIMEOUT = 1 << 0
FAULT_ESTOP = 1 << 1
FAULT_SERVO_FAULT = 1 << 2
FAULT_BROWNOUT = 1 << 3


class ProtocolError(ValueError):
    """Malformed wire data, an unsupported protocol version, or an
    out-of-range field -- at construction time or decode time alike."""


class CommandType(str, Enum):
    WALK = "walk"
    TURN = "turn"
    STOP = "stop"
    BODY_HEIGHT = "body_height"
    PAN_TILT = "pan_tilt"
    FACE = "face"
    CALIBRATE = "calibrate"
    CALIBRATION_MODE = "calibration_mode"
    READ_OFFSETS = "read_offsets"
    WRITE_OFFSETS = "write_offsets"
    PING = "ping"


class Mood(str, Enum):
    """Provisional set -- no OLED face implemented yet."""

    NEUTRAL = "neutral"
    HAPPY = "happy"
    ANGRY = "angry"
    SURPRISED = "surprised"
    SLEEPY = "sleepy"


def has_fault(fault_flags: int, bit: int) -> bool:
    return bool(fault_flags & bit)


def _require_range(name: str, value, lo: float, hi: float) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ProtocolError(f"{name} must be a number, got {value!r}")
    if not (lo <= value <= hi):
        raise ProtocolError(f"{name}={value!r} out of range [{lo}, {hi}]")


def _require_int_range(name: str, value, lo: int, hi: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ProtocolError(f"{name} must be an int, got {value!r}")
    if not (lo <= value <= hi):
        raise ProtocolError(f"{name}={value!r} out of range [{lo}, {hi}]")


def _field(fields: dict, name: str):
    try:
        return fields[name]
    except KeyError:
        raise ProtocolError(f"missing field {name!r}") from None


# --- Commands (PC -> robot) -------------------------------------------------


class Command(ABC):
    """Base for all outbound intent commands. Carries no transport metadata
    (sequence number, protocol version) -- those are assigned at encode
    time by the sending link, not by whoever constructs the intent."""

    TYPE: ClassVar[CommandType]

    @abstractmethod
    def _wire_fields(self) -> dict:
        """Type-specific payload fields, excluding v/seq/type."""
        raise NotImplementedError


@dataclass(frozen=True)
class WalkCommand(Command):
    TYPE: ClassVar[CommandType] = CommandType.WALK
    vx: float
    vy: float
    speed: float

    def __post_init__(self) -> None:
        _require_range("vx", self.vx, -1.0, 1.0)
        _require_range("vy", self.vy, -1.0, 1.0)
        _require_range("speed", self.speed, 0.0, 100.0)

    def _wire_fields(self) -> dict:
        return {"vx": self.vx, "vy": self.vy, "speed": self.speed}


@dataclass(frozen=True)
class TurnCommand(Command):
    TYPE: ClassVar[CommandType] = CommandType.TURN
    rate: float
    speed: float

    def __post_init__(self) -> None:
        _require_range("rate", self.rate, -1.0, 1.0)
        _require_range("speed", self.speed, 0.0, 100.0)

    def _wire_fields(self) -> dict:
        return {"rate": self.rate, "speed": self.speed}


@dataclass(frozen=True)
class StopCommand(Command):
    TYPE: ClassVar[CommandType] = CommandType.STOP

    def _wire_fields(self) -> dict:
        return {}


@dataclass(frozen=True)
class BodyHeightCommand(Command):
    TYPE: ClassVar[CommandType] = CommandType.BODY_HEIGHT
    height: float

    def __post_init__(self) -> None:
        _require_range("height", self.height, 0.0, 100.0)

    def _wire_fields(self) -> dict:
        return {"height": self.height}


@dataclass(frozen=True)
class PanTiltCommand(Command):
    TYPE: ClassVar[CommandType] = CommandType.PAN_TILT
    pan: float
    tilt: float

    def __post_init__(self) -> None:
        _require_range("pan", self.pan, PAN_TILT_MIN_DEG, PAN_TILT_MAX_DEG)
        _require_range("tilt", self.tilt, PAN_TILT_MIN_DEG, PAN_TILT_MAX_DEG)

    def _wire_fields(self) -> dict:
        return {"pan": self.pan, "tilt": self.tilt}


@dataclass(frozen=True)
class FaceCommand(Command):
    TYPE: ClassVar[CommandType] = CommandType.FACE
    mood: Mood

    def __post_init__(self) -> None:
        if not isinstance(self.mood, Mood):
            raise ProtocolError(f"mood must be a Mood, got {self.mood!r}")

    def _wire_fields(self) -> dict:
        return {"mood": self.mood.value}


@dataclass(frozen=True)
class CalibrateCommand(Command):
    """Single-servo trim write. Servo-scoped, not leg-scoped -- see
    docs/protocol.md's note on ANALYSIS.md Section 3. Requires calibration
    mode to be armed; see CalibrationModeCommand."""

    TYPE: ClassVar[CommandType] = CommandType.CALIBRATE
    servo_index: int
    offset_us: int
    sign: int = 1

    def __post_init__(self) -> None:
        _require_int_range("servo_index", self.servo_index, 0, SERVO_COUNT - 1)
        _require_int_range(
            "offset_us",
            self.offset_us,
            -CALIBRATION_OFFSET_LIMIT_US,
            CALIBRATION_OFFSET_LIMIT_US,
        )
        if self.sign not in (-1, 1):
            raise ProtocolError(f"sign must be -1 or 1, got {self.sign!r}")

    def _wire_fields(self) -> dict:
        return {
            "servo_index": self.servo_index,
            "offset_us": self.offset_us,
            "sign": self.sign,
        }


@dataclass(frozen=True)
class ServoOffset:
    """One entry of the 18-servo calibration table. Used in WriteOffsetsCommand
    and in Telemetry.offsets (the read_offsets reply)."""

    servo_index: int
    offset_us: int
    sign: int = 1

    def __post_init__(self) -> None:
        _require_int_range("servo_index", self.servo_index, 0, SERVO_COUNT - 1)
        _require_int_range(
            "offset_us",
            self.offset_us,
            -CALIBRATION_OFFSET_LIMIT_US,
            CALIBRATION_OFFSET_LIMIT_US,
        )
        if self.sign not in (-1, 1):
            raise ProtocolError(f"sign must be -1 or 1, got {self.sign!r}")

    def to_dict(self) -> dict:
        return {
            "servo_index": self.servo_index,
            "offset_us": self.offset_us,
            "sign": self.sign,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ServoOffset":
        if not isinstance(data, dict):
            raise ProtocolError(f"offset entry must be an object, got {data!r}")
        try:
            return cls(
                servo_index=data["servo_index"],
                offset_us=data["offset_us"],
                sign=data.get("sign", 1),
            )
        except KeyError as exc:
            raise ProtocolError(f"offset entry missing field {exc}") from None


@dataclass(frozen=True)
class CalibrationModeCommand(Command):
    """Arms or disarms calibrate/write_offsets writes. Auto-disarms after
    CALIBRATION_ARM_TIMEOUT_S of no calibration-related traffic -- see
    docs/protocol.md Section 6."""

    TYPE: ClassVar[CommandType] = CommandType.CALIBRATION_MODE
    armed: bool

    def __post_init__(self) -> None:
        if not isinstance(self.armed, bool):
            raise ProtocolError(f"armed must be a bool, got {self.armed!r}")

    def _wire_fields(self) -> dict:
        return {"armed": self.armed}


@dataclass(frozen=True)
class ReadOffsetsCommand(Command):
    """Requests the full 18-servo calibration table back via telemetry.
    Always allowed, armed or not -- reading is not a write."""

    TYPE: ClassVar[CommandType] = CommandType.READ_OFFSETS

    def _wire_fields(self) -> dict:
        return {}


@dataclass(frozen=True)
class WriteOffsetsCommand(Command):
    """Bulk restore of the calibration table, e.g. from a GUI-exported
    YAML file. Requires calibration mode armed, same as CalibrateCommand.
    Exists so a bad calibration session is a five-second bulk restore, not
    18 individual re-sends."""

    TYPE: ClassVar[CommandType] = CommandType.WRITE_OFFSETS
    offsets: tuple

    def __post_init__(self) -> None:
        object.__setattr__(self, "offsets", tuple(self.offsets))
        if not self.offsets:
            raise ProtocolError("write_offsets requires at least one offset entry")
        if len(self.offsets) > SERVO_COUNT:
            raise ProtocolError(
                f"write_offsets got {len(self.offsets)} entries, max {SERVO_COUNT}"
            )
        if not all(isinstance(o, ServoOffset) for o in self.offsets):
            raise ProtocolError("write_offsets entries must be ServoOffset")
        indices = [o.servo_index for o in self.offsets]
        if len(indices) != len(set(indices)):
            raise ProtocolError("write_offsets has duplicate servo_index entries")

    def _wire_fields(self) -> dict:
        return {"offsets": [o.to_dict() for o in self.offsets]}


@dataclass(frozen=True)
class PingCommand(Command):
    TYPE: ClassVar[CommandType] = CommandType.PING

    def _wire_fields(self) -> dict:
        return {}


_COMMAND_DECODERS: dict[str, Callable[[dict], Command]] = {}


def _register(command_type: CommandType):
    def decorator(fn: Callable[[dict], Command]) -> Callable[[dict], Command]:
        _COMMAND_DECODERS[command_type.value] = fn
        return fn

    return decorator


@_register(CommandType.WALK)
def _decode_walk(fields: dict) -> Command:
    return WalkCommand(
        vx=_field(fields, "vx"), vy=_field(fields, "vy"), speed=_field(fields, "speed")
    )


@_register(CommandType.TURN)
def _decode_turn(fields: dict) -> Command:
    return TurnCommand(rate=_field(fields, "rate"), speed=_field(fields, "speed"))


@_register(CommandType.STOP)
def _decode_stop(fields: dict) -> Command:
    return StopCommand()


@_register(CommandType.BODY_HEIGHT)
def _decode_body_height(fields: dict) -> Command:
    return BodyHeightCommand(height=_field(fields, "height"))


@_register(CommandType.PAN_TILT)
def _decode_pan_tilt(fields: dict) -> Command:
    return PanTiltCommand(pan=_field(fields, "pan"), tilt=_field(fields, "tilt"))


@_register(CommandType.FACE)
def _decode_face(fields: dict) -> Command:
    raw_mood = _field(fields, "mood")
    try:
        mood = Mood(raw_mood)
    except ValueError:
        raise ProtocolError(f"unknown mood {raw_mood!r}") from None
    return FaceCommand(mood=mood)


@_register(CommandType.CALIBRATE)
def _decode_calibrate(fields: dict) -> Command:
    return CalibrateCommand(
        servo_index=_field(fields, "servo_index"),
        offset_us=_field(fields, "offset_us"),
        sign=fields.get("sign", 1),
    )


@_register(CommandType.CALIBRATION_MODE)
def _decode_calibration_mode(fields: dict) -> Command:
    return CalibrationModeCommand(armed=_field(fields, "armed"))


@_register(CommandType.READ_OFFSETS)
def _decode_read_offsets(fields: dict) -> Command:
    return ReadOffsetsCommand()


@_register(CommandType.WRITE_OFFSETS)
def _decode_write_offsets(fields: dict) -> Command:
    raw_offsets = _field(fields, "offsets")
    if not isinstance(raw_offsets, list):
        raise ProtocolError("offsets must be a list")
    return WriteOffsetsCommand(offsets=tuple(ServoOffset.from_dict(o) for o in raw_offsets))


@_register(CommandType.PING)
def _decode_ping(fields: dict) -> Command:
    return PingCommand()


@dataclass(frozen=True)
class ReceivedCommand:
    seq: int
    command: Command


def _parse_envelope(data: bytes) -> dict:
    try:
        payload = json.loads(data)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ProtocolError(f"malformed JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ProtocolError(f"payload must be a JSON object, got {type(payload).__name__}")
    version = payload.get("v")
    if version != PROTOCOL_VERSION:
        raise ProtocolError(
            f"unsupported protocol version {version!r}, expected {PROTOCOL_VERSION}"
        )
    return payload


def _require_seq(value) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not (0 <= value < SEQUENCE_MODULUS):
        raise ProtocolError(f"invalid sequence number: {value!r}")
    return value


def encode_command(command: Command, seq: int) -> bytes:
    _require_seq(seq)
    payload = {"v": PROTOCOL_VERSION, "seq": seq, "type": command.TYPE.value}
    payload.update(command._wire_fields())
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_command(data: bytes) -> ReceivedCommand:
    payload = _parse_envelope(data)
    seq = _require_seq(payload.get("seq"))
    type_name = payload.get("type")
    decoder = _COMMAND_DECODERS.get(type_name)
    if decoder is None:
        raise ProtocolError(f"unknown command type {type_name!r}")
    fields = {k: v for k, v in payload.items() if k not in ("v", "seq", "type")}
    command = decoder(fields)
    return ReceivedCommand(seq=seq, command=command)


def _decode_command_dict(raw: dict) -> Command:
    type_name = raw.get("type")
    decoder = _COMMAND_DECODERS.get(type_name)
    if decoder is None:
        raise ProtocolError(f"unknown command type {type_name!r}")
    fields = {k: v for k, v in raw.items() if k != "type"}
    return decoder(fields)


# --- Telemetry (robot -> PC) -------------------------------------------------


@dataclass(frozen=True)
class Telemetry:
    """Sent as a synchronous reply to every command the robot processes --
    see docs/protocol.md Section 3 for why there's no independent send
    schedule. `last_applied` reflects motion/pose state only (walk, turn,
    stop, body_height, pan_tilt, face); calibration-family commands report
    through `ok`/`error`/`offsets` instead, not through `last_applied`."""

    seq_echo: int
    ok: bool
    error: str | None
    fault_flags: int
    gait_phase: float | None
    rail_mv: int | None
    link_timeout_s: float
    calibration_armed: bool
    last_applied: Command | None
    offsets: tuple | None


def encode_telemetry(telemetry: Telemetry) -> bytes:
    payload = {
        "v": PROTOCOL_VERSION,
        "seq_echo": telemetry.seq_echo,
        "ok": telemetry.ok,
        "error": telemetry.error,
        "fault_flags": telemetry.fault_flags,
        "gait_phase": telemetry.gait_phase,
        "rail_mv": telemetry.rail_mv,
        "link_timeout_s": telemetry.link_timeout_s,
        "calibration_armed": telemetry.calibration_armed,
        "last_applied": (
            {"type": telemetry.last_applied.TYPE.value, **telemetry.last_applied._wire_fields()}
            if telemetry.last_applied is not None
            else None
        ),
        "offsets": (
            [o.to_dict() for o in telemetry.offsets] if telemetry.offsets is not None else None
        ),
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_telemetry(data: bytes) -> Telemetry:
    payload = _parse_envelope(data)

    seq_echo = _require_seq(payload.get("seq_echo"))

    try:
        ok = payload["ok"]
        fault_flags = payload["fault_flags"]
        link_timeout_s = payload["link_timeout_s"]
        calibration_armed = payload["calibration_armed"]
    except KeyError as exc:
        raise ProtocolError(f"telemetry missing field {exc}") from None

    if not isinstance(ok, bool):
        raise ProtocolError(f"ok must be a bool, got {ok!r}")
    if not isinstance(fault_flags, int) or isinstance(fault_flags, bool) or fault_flags < 0:
        raise ProtocolError(f"invalid fault_flags: {fault_flags!r}")
    if not isinstance(calibration_armed, bool):
        raise ProtocolError(f"calibration_armed must be a bool, got {calibration_armed!r}")
    if not isinstance(link_timeout_s, (int, float)) or isinstance(link_timeout_s, bool):
        raise ProtocolError(f"link_timeout_s must be a number, got {link_timeout_s!r}")

    error = payload.get("error")
    if error is not None and not isinstance(error, str):
        raise ProtocolError(f"error must be a string or null, got {error!r}")

    gait_phase = payload.get("gait_phase")
    if gait_phase is not None:
        if (
            not isinstance(gait_phase, (int, float))
            or isinstance(gait_phase, bool)
            or not (0.0 <= gait_phase <= 1.0)
        ):
            raise ProtocolError(f"gait_phase out of range: {gait_phase!r}")

    rail_mv = payload.get("rail_mv")
    if rail_mv is not None and (not isinstance(rail_mv, int) or isinstance(rail_mv, bool)):
        raise ProtocolError(f"rail_mv must be an int or null, got {rail_mv!r}")

    raw_offsets = payload.get("offsets")
    offsets = None
    if raw_offsets is not None:
        if not isinstance(raw_offsets, list):
            raise ProtocolError("offsets must be a list or null")
        offsets = tuple(ServoOffset.from_dict(o) for o in raw_offsets)

    raw_last_applied = payload.get("last_applied")
    last_applied = _decode_command_dict(raw_last_applied) if raw_last_applied is not None else None

    return Telemetry(
        seq_echo=seq_echo,
        ok=ok,
        error=error,
        fault_flags=fault_flags,
        gait_phase=float(gait_phase) if gait_phase is not None else None,
        rail_mv=rail_mv,
        link_timeout_s=float(link_timeout_s),
        calibration_armed=calibration_armed,
        last_applied=last_applied,
        offsets=offsets,
    )


# --- Sequence numbers ---------------------------------------------------


def next_sequence(seq: int) -> int:
    return (seq + 1) % SEQUENCE_MODULUS


def sequence_is_newer(candidate: int, reference: int) -> bool:
    """RFC 1982-style wraparound-safe comparison: True if `candidate` is
    newer than `reference` in the SEQUENCE_MODULUS sequence space. Equal
    counts as not-newer (a duplicate, not an update)."""
    diff = (candidate - reference) % SEQUENCE_MODULUS
    return 0 < diff < (SEQUENCE_MODULUS // 2)
