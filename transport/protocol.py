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

# Bench mode: raw pulse to a physical PCA9685 channel, no kinematics, no
# servo_index (a loose servo hasn't been assigned a leg position yet). See
# docs/protocol.md Section 8 for why this is a separate command family with
# its own arm gate rather than an extension of calibrate.
PCA9685_BOARD_ADDRESSES = (0x40, 0x41)
PCA9685_CHANNELS_PER_BOARD = 16
BENCH_PULSE_MIN_US = 500
BENCH_PULSE_MAX_US = 2500
NEUTRAL_PULSE_US = 1500  # nominal hobby-servo center; per-unit correction is offset_us, applied later
# Coxa/femur are bipolar around NEUTRAL_PULSE_US (servo command 90 = straight);
# tibia is zero-based -- its horn is mounted so 0, not 90, is straight
# (ANALYSIS.md Section 2). Mirrors firmware/include/Config.h's
# TIBIA_NEUTRAL_PULSE_US -- same hand-copied-with-cross-reference pattern
# as NEUTRAL_PULSE_US/BENCH_PULSE_MIN_US/MAX_US above (see
# scripts/gen_protocol_constants.py's docstring for why this isn't
# codegen'd: a mismatch here is a rejected command, not a misfired
# failsafe). 645us is the target under docs/HOW_TO_USE.md's mounting
# procedure (servo commanded to its own 1500us center while the tibia is
# held at its gait-envelope-midpoint fold, ~77 degrees) -- not yet a real
# measurement; see Config.h's copy of this constant for the derivation
# and the required post-mount verification.
TIBIA_NEUTRAL_PULSE_US = 645
HEALTH_NOTE_MAX_LEN = 500

_US_PER_DEG = (BENCH_PULSE_MAX_US - BENCH_PULSE_MIN_US) / 180.0


def neutral_pulse_us_for_servo(servo_index: int) -> int:
    """Coxa/femur/tibia = servo_index % 3, matching ui/servo_names.py's
    documented placeholder convention (real wiring comes from a future
    SERVO_MAP, not yet assigned -- CLAUDE.md) and firmware's
    ServoMap.h/neutralPulseUsFor(JointType). Joint index 2 is tibia;
    0 and 1 (coxa, femur) share the bipolar convention."""
    return TIBIA_NEUTRAL_PULSE_US if servo_index % 3 == 2 else NEUTRAL_PULSE_US


def pulse_us_to_deg_from_neutral(pulse_us: int, servo_index: int) -> float:
    """Nominal, uncalibrated conversion -- no sign/offset_us applied,
    mirroring firmware's AngleToPulse::pulseUsToDegFromNeutral and bench
    mode's own raw-pulse semantics (bench_pulse never applies a servo's
    calibration, see docs/protocol.md Section 8). A bench-recorded
    mechanical limit must transfer to every unit of this joint type
    regardless of that specific unit's spline-mounting calibration --
    see ServoProfile.min_deg_from_neutral."""
    return (pulse_us - neutral_pulse_us_for_servo(servo_index)) / _US_PER_DEG


def neutral_servo_deg_for_servo(servo_index: int) -> float:
    """Coxa/femur bipolar-90, tibia zero-based -- same servo_index % 3
    convention as neutral_pulse_us_for_servo, mirrors firmware's
    ServoMap.h/neutralServoDegFor(JointType) and
    robot/kinematics.py's to_servo_deg()."""
    return 0.0 if servo_index % 3 == 2 else 90.0


def angle_to_pulse_us(servo_deg: float, neutral_servo_deg: float, neutral_pulse_us: int, sign: int, offset_us: int) -> int:
    """Mirrors firmware's AngleToPulse::angleToPulseUs exactly (same
    formula, same clamp) -- the PC-side half of this project's "two
    independent implementations, kept in lockstep" pattern
    (ANALYSIS.md Section 7), needed so the Test Leg tab's IK mode can
    turn a computed joint angle into an actual bench_pulse command
    without a compiled extension. Clamped to
    [BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US] defensively before ever
    reaching a wire command -- BenchPulseCommand's own construction-time
    validation would reject an out-of-range value anyway, but this
    keeps the two clamps consistent with firmware's own belt-and-braces
    approach rather than relying on that validation as the only line of
    defense."""
    pulse = neutral_pulse_us + sign * (servo_deg - neutral_servo_deg) * _US_PER_DEG + offset_us
    pulse = max(BENCH_PULSE_MIN_US, min(BENCH_PULSE_MAX_US, pulse))
    return round(pulse)


FAULT_NONE = 0
FAULT_LINK_TIMEOUT = 1 << 0
FAULT_ESTOP = 1 << 1
FAULT_SERVO_FAULT = 1 << 2
FAULT_BROWNOUT = 1 << 3
# Set when the gait control loop's most recent tick needed to clamp
# either a leg's D (reachability) or a joint angle -- level-triggered,
# cleared the next tick if it doesn't recur. See Telemetry.ik_clip_count
# below for the cumulative counters this summarizes.
FAULT_IK_CLIP = 1 << 4


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
    BENCH_MODE = "bench_mode"
    BENCH_PULSE = "bench_pulse"
    BENCH_RELEASE = "bench_release"
    RECORD_LIMIT = "record_limit"
    BENCH_HEALTH_NOTE = "bench_health_note"
    PING = "ping"


class Mood(str, Enum):
    """Provisional set -- no OLED face implemented yet."""

    NEUTRAL = "neutral"
    HAPPY = "happy"
    ANGRY = "angry"
    SURPRISED = "surprised"
    SLEEPY = "sleepy"


class LimitBound(str, Enum):
    MIN = "min"
    MAX = "max"


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


def _require_optional_int_range(name: str, value, lo: int, hi: int) -> None:
    if value is None:
        return
    _require_int_range(name, value, lo, hi)


def _require_optional_range(name: str, value, lo: float, hi: float) -> None:
    if value is None:
        return
    _require_range(name, value, lo, hi)


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
    docs/protocol.md's note on ANALYSIS.md Section 3. Only ever touches
    offset_us/sign, never the pulse limits (see ServoProfile) -- those are
    bench mode's concern, written only by RecordLimitCommand. Requires
    calibration mode to be armed; see CalibrationModeCommand."""

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
class ServoProfile:
    """Everything known about one physical servo, keyed by servo_index.
    offset_us/sign are a correction, written only via CalibrateCommand/
    WriteOffsetsCommand (calibration_mode gated). min_deg_from_neutral/
    max_deg_from_neutral are a safety bound the firmware is meant to
    enforce on every command regardless of source, written only via
    RecordLimitCommand (bench_mode gated) -- both live in one record
    because they describe the same unit and because splitting them risks
    the export/import files silently drifting apart, but the two write
    paths stay separate on purpose (see docs/protocol.md Section 1). note
    is a free-text health note, ungated since it's advisory record-
    keeping, not something the firmware acts on.

    Degrees from this joint's own neutral (0 = neutral, matching
    AngleToPulse's servoDeg-minus-neutralServoDeg convention), not
    absolute pulse microseconds -- deliberately, so a limit measured once
    on the single test leg (reference/ANALYSIS.md's bring-up plan)
    transfers to all six servos of that joint type, each with its own
    offset_us. offset_us's whole job is making "commanded at neutral"
    mean the same physical pose on every unit regardless of that unit's
    spline-mounting error, so a mechanical bound expressed relative to
    neutral is unit-independent in exactly the way an absolute pulse
    isn't -- see pulse_us_to_deg_from_neutral()'s docstring. Recorded via
    RecordLimitCommand's pulse_us (what the operator actually verifies
    safe on the bench, in raw hardware terms) and converted at the point
    it's merged into a profile, not on the wire -- keeping pulse<->degree
    conversion out of the wire format matches ANALYSIS.md Section 7's
    "one leaf function" rule for pulse units in general.

    None until a bench session has recorded a bound -- that must be
    distinguishable from "recorded as 0", so this is not defaulted to a
    fake safe-looking number.
    """

    servo_index: int
    offset_us: int = 0
    sign: int = 1
    min_deg_from_neutral: float | None = None
    max_deg_from_neutral: float | None = None
    note: str = ""

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
        # The valid degree-from-neutral range is joint-type-dependent --
        # a tibia servo's neutral sits at one end of its pulse range
        # (zero-based), not the middle (bipolar), so its legal range
        # relative to neutral is [0, +180], not coxa/femur's [-90, +90].
        deg_lo = pulse_us_to_deg_from_neutral(BENCH_PULSE_MIN_US, self.servo_index)
        deg_hi = pulse_us_to_deg_from_neutral(BENCH_PULSE_MAX_US, self.servo_index)
        _require_optional_range("min_deg_from_neutral", self.min_deg_from_neutral, deg_lo, deg_hi)
        _require_optional_range("max_deg_from_neutral", self.max_deg_from_neutral, deg_lo, deg_hi)
        if (
            self.min_deg_from_neutral is not None
            and self.max_deg_from_neutral is not None
            and self.min_deg_from_neutral >= self.max_deg_from_neutral
        ):
            raise ProtocolError(
                f"min_deg_from_neutral ({self.min_deg_from_neutral}) must be < "
                f"max_deg_from_neutral ({self.max_deg_from_neutral})"
            )
        if not isinstance(self.note, str):
            raise ProtocolError(f"note must be a string, got {self.note!r}")
        if len(self.note) > HEALTH_NOTE_MAX_LEN:
            raise ProtocolError(f"note exceeds {HEALTH_NOTE_MAX_LEN} characters")

    def to_dict(self) -> dict:
        return {
            "servo_index": self.servo_index,
            "offset_us": self.offset_us,
            "sign": self.sign,
            "min_deg_from_neutral": self.min_deg_from_neutral,
            "max_deg_from_neutral": self.max_deg_from_neutral,
            "note": self.note,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ServoProfile":
        if not isinstance(data, dict):
            raise ProtocolError(f"profile entry must be an object, got {data!r}")
        try:
            return cls(
                servo_index=data["servo_index"],
                offset_us=data.get("offset_us", 0),
                sign=data.get("sign", 1),
                min_deg_from_neutral=data.get("min_deg_from_neutral"),
                max_deg_from_neutral=data.get("max_deg_from_neutral"),
                note=data.get("note", ""),
            )
        except KeyError as exc:
            raise ProtocolError(f"profile entry missing field {exc}") from None


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
    """Requests the full servo profile table back via telemetry (offsets,
    limits, and health notes for every servo_index that has any recorded).
    Always allowed, armed or not -- reading is not a write."""

    TYPE: ClassVar[CommandType] = CommandType.READ_OFFSETS

    def _wire_fields(self) -> dict:
        return {}


@dataclass(frozen=True)
class WriteOffsetsCommand(Command):
    """Bulk restore of offset_us/sign, e.g. from a GUI-exported YAML file.
    Requires calibration mode armed, same as CalibrateCommand. Exists so a
    bad calibration session is a five-second bulk restore, not 18
    individual re-sends. Does not touch min_deg_from_neutral/
    max_deg_from_neutral/note even though ServoProfile carries them --
    those are bench mode's concern."""

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
        if not all(isinstance(o, ServoProfile) for o in self.offsets):
            raise ProtocolError("write_offsets entries must be ServoProfile")
        indices = [o.servo_index for o in self.offsets]
        if len(indices) != len(set(indices)):
            raise ProtocolError("write_offsets has duplicate servo_index entries")

    def _wire_fields(self) -> dict:
        return {
            "offsets": [
                {"servo_index": o.servo_index, "offset_us": o.offset_us, "sign": o.sign}
                for o in self.offsets
            ]
        }


@dataclass(frozen=True)
class BenchModeCommand(Command):
    """Arms or disarms bench_pulse/record_limit. Separate from
    calibration_mode on purpose -- see docs/protocol.md Section 8: a stray
    calibrate packet corrupts a trim number, a stray bench_pulse packet
    drives a channel directly with no kinematics or clamping in the way,
    which is a materially worse failure mode and earns its own switch.
    Auto-disarms after BENCH_ARM_TIMEOUT_S of no bench-related traffic."""

    TYPE: ClassVar[CommandType] = CommandType.BENCH_MODE
    armed: bool

    def __post_init__(self) -> None:
        if not isinstance(self.armed, bool):
            raise ProtocolError(f"armed must be a bool, got {self.armed!r}")

    def _wire_fields(self) -> dict:
        return {"armed": self.armed}


@dataclass(frozen=True)
class BenchPulseCommand(Command):
    """Direct pulse to one physical PCA9685 channel. No kinematics --
    board/channel is "what am I driving", independent of servo_index
    ("what am I recording"), see docs/protocol.md Section 8. servo_index
    is optional: a loose bench servo hasn't necessarily been assigned a
    leg position yet. When given, it lets the receiver convert this pulse
    to degrees from that servo's own neutral and enforce that servo's
    recorded min/max_deg_from_neutral (if any) against it -- firmware's
    GatedServoDriver does this unconditionally; MockRobotLink mirrors it
    for parity (see transport/mock_link.py). Bounds on pulse_us are the
    generic hobby-servo envelope, not a per-unit safety limit; the GUI's
    slider does not currently narrow itself to a servo's recorded limit,
    only the wire enforcement described above does. Requires bench mode
    armed."""

    TYPE: ClassVar[CommandType] = CommandType.BENCH_PULSE
    board: int
    channel: int
    pulse_us: int
    servo_index: int | None = None

    def __post_init__(self) -> None:
        if self.board not in PCA9685_BOARD_ADDRESSES:
            raise ProtocolError(
                f"board={self.board!r} not in {PCA9685_BOARD_ADDRESSES}"
            )
        _require_int_range("channel", self.channel, 0, PCA9685_CHANNELS_PER_BOARD - 1)
        _require_int_range("pulse_us", self.pulse_us, BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US)
        _require_optional_int_range("servo_index", self.servo_index, 0, SERVO_COUNT - 1)

    def _wire_fields(self) -> dict:
        fields = {"board": self.board, "channel": self.channel, "pulse_us": self.pulse_us}
        if self.servo_index is not None:
            fields["servo_index"] = self.servo_index
        return fields


@dataclass(frozen=True)
class BenchReleaseCommand(Command):
    """Goes limp: the PCA9685 stops outputting a pulse on this channel
    entirely, the same as GatedServoDriver.release() (SafeState.h's
    Bench-mode fault path, or the serial mirror's "bench release", which
    predates this wire command and drove the same underlying call). Not
    "park at neutral" -- that still commands a pulse and holds against
    it; this commands nothing. Always reachable while bench mode is
    armed, independent of any recorded limit -- there's nothing to
    validate about going limp. board/channel addressed like
    BenchPulseCommand ("what am I driving"), no servo_index: releasing
    doesn't touch a profile, so there's nothing to look up one for."""

    TYPE: ClassVar[CommandType] = CommandType.BENCH_RELEASE
    board: int
    channel: int

    def __post_init__(self) -> None:
        if self.board not in PCA9685_BOARD_ADDRESSES:
            raise ProtocolError(f"board={self.board!r} not in {PCA9685_BOARD_ADDRESSES}")
        _require_int_range("channel", self.channel, 0, PCA9685_CHANNELS_PER_BOARD - 1)

    def _wire_fields(self) -> dict:
        return {"board": self.board, "channel": self.channel}


@dataclass(frozen=True)
class RecordLimitCommand(Command):
    """Records a bench-discovered safe pulse bound into servo_index's
    profile. servo_index-scoped (not board/channel) because this is what
    persists once the loose servo being bench-tested is assigned to a
    future leg position. Requires bench mode armed."""

    TYPE: ClassVar[CommandType] = CommandType.RECORD_LIMIT
    servo_index: int
    bound: LimitBound
    pulse_us: int

    def __post_init__(self) -> None:
        _require_int_range("servo_index", self.servo_index, 0, SERVO_COUNT - 1)
        if not isinstance(self.bound, LimitBound):
            raise ProtocolError(f"bound must be a LimitBound, got {self.bound!r}")
        _require_int_range("pulse_us", self.pulse_us, BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US)

    def _wire_fields(self) -> dict:
        return {"servo_index": self.servo_index, "bound": self.bound.value, "pulse_us": self.pulse_us}


@dataclass(frozen=True)
class BenchHealthNoteCommand(Command):
    """Free-text health note for servo_index ("buzzes at low end", "dead").
    Not gated by bench mode -- it's advisory record-keeping, not a physical
    actuation or a safety-relevant value, and you may want to note a unit
    that isn't the one currently plugged into the bench rig."""

    TYPE: ClassVar[CommandType] = CommandType.BENCH_HEALTH_NOTE
    servo_index: int
    note: str

    def __post_init__(self) -> None:
        _require_int_range("servo_index", self.servo_index, 0, SERVO_COUNT - 1)
        if not isinstance(self.note, str):
            raise ProtocolError(f"note must be a string, got {self.note!r}")
        if len(self.note) > HEALTH_NOTE_MAX_LEN:
            raise ProtocolError(f"note exceeds {HEALTH_NOTE_MAX_LEN} characters")

    def _wire_fields(self) -> dict:
        return {"servo_index": self.servo_index, "note": self.note}


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
    return WriteOffsetsCommand(offsets=tuple(ServoProfile.from_dict(o) for o in raw_offsets))


@_register(CommandType.BENCH_MODE)
def _decode_bench_mode(fields: dict) -> Command:
    return BenchModeCommand(armed=_field(fields, "armed"))


@_register(CommandType.BENCH_PULSE)
def _decode_bench_pulse(fields: dict) -> Command:
    return BenchPulseCommand(
        board=_field(fields, "board"),
        channel=_field(fields, "channel"),
        pulse_us=_field(fields, "pulse_us"),
        servo_index=fields.get("servo_index"),
    )


@_register(CommandType.BENCH_RELEASE)
def _decode_bench_release(fields: dict) -> Command:
    return BenchReleaseCommand(board=_field(fields, "board"), channel=_field(fields, "channel"))


@_register(CommandType.RECORD_LIMIT)
def _decode_record_limit(fields: dict) -> Command:
    raw_bound = _field(fields, "bound")
    try:
        bound = LimitBound(raw_bound)
    except ValueError:
        raise ProtocolError(f"unknown bound {raw_bound!r}") from None
    return RecordLimitCommand(
        servo_index=_field(fields, "servo_index"),
        bound=bound,
        pulse_us=_field(fields, "pulse_us"),
    )


@_register(CommandType.BENCH_HEALTH_NOTE)
def _decode_bench_health_note(fields: dict) -> Command:
    return BenchHealthNoteCommand(
        servo_index=_field(fields, "servo_index"), note=_field(fields, "note")
    )


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
    stop, body_height, pan_tilt, face); calibration- and bench-family
    commands report through `ok`/`error`/`profiles` instead, not through
    `last_applied` -- bench_pulse in particular is not motion/pose state,
    it's direct hardware I/O with no kinematic meaning.

    `robot_assembled` echoes firmware's compiled-in ROBOT_ASSEMBLED build
    flag, same reporting pattern as `link_timeout_s` -- but unlike
    `link_timeout_s`, there is no PC-side "expected" value to cross-check
    it against (RobotLink.constants_warning has nothing to compare this
    to), and no sensor anywhere in this system that observes physical
    assembly state to detect a stale flag either. This field is reported
    so the GUI can display it, not because a mismatch is detectable --
    see docs/protocol.md Section 9 for why that was considered and
    rejected, not just left undone."""

    seq_echo: int
    ok: bool
    error: str | None
    fault_flags: int
    gait_phase: float | None
    rail_mv: int | None
    link_timeout_s: float
    calibration_armed: bool
    bench_armed: bool
    robot_assembled: bool
    last_applied: Command | None
    profiles: tuple | None
    # Cumulative since the gait engine last reset (boot, or -- once it
    # exists -- SimRobotLink construction), never since the last packet.
    # ANALYSIS.md Section 5.7 flagged IK's reachability clamp and joint-
    # angle clamp as silent; these are what make them observable instead
    # of discarded. Default 0/0.0 rather than nullable: unlike gait_phase
    # (meaningless while idle), "zero clips so far" is a real, always-
    # meaningful value, including for MockRobotLink, which never runs
    # gait at all. See robot/gait.py's GaitState docstring for the full
    # design rationale, including why FAULT_IK_CLIP (this tick only) is
    # a separate signal from these cumulative counters.
    ik_clip_count: int = 0
    ik_clip_worst_mm: float = 0.0
    joint_clip_count: int = 0
    joint_clip_worst_deg: float = 0.0
    # Inbound-packet accounting, cumulative since the robot booted --
    # docs/protocol.md Section 4. stale_drop_count counts packets
    # rejected by the receiver's sequence-freshness rule;
    # malformed_drop_count counts packets that failed to decode at all.
    # Together they answer "are my packets arriving, and if so what is
    # the robot doing with them" -- the question a red LINK light alone
    # cannot answer, and which previously needed a serial cable and a
    # firmware source read. Always 0 from MockRobotLink/SimRobotLink for
    # malformed_drop_count specifically: there is no wire to malform.
    stale_drop_count: int = 0
    malformed_drop_count: int = 0
    # The sequence number the receiver will compare the next packet
    # against. None until it has accepted one -- 0 is a real sequence
    # number and can't double as "nothing yet". A value far above what
    # this process is currently sending means the robot is rejecting
    # everything as stale (see transport/link_watchdog.py).
    last_accepted_seq: int | None = None


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
        "bench_armed": telemetry.bench_armed,
        "robot_assembled": telemetry.robot_assembled,
        "ik_clip_count": telemetry.ik_clip_count,
        "ik_clip_worst_mm": telemetry.ik_clip_worst_mm,
        "joint_clip_count": telemetry.joint_clip_count,
        "joint_clip_worst_deg": telemetry.joint_clip_worst_deg,
        "stale_drop_count": telemetry.stale_drop_count,
        "malformed_drop_count": telemetry.malformed_drop_count,
        "last_accepted_seq": telemetry.last_accepted_seq,
        "last_applied": (
            {"type": telemetry.last_applied.TYPE.value, **telemetry.last_applied._wire_fields()}
            if telemetry.last_applied is not None
            else None
        ),
        "profiles": (
            [p.to_dict() for p in telemetry.profiles] if telemetry.profiles is not None else None
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
        bench_armed = payload["bench_armed"]
        robot_assembled = payload["robot_assembled"]
        ik_clip_count = payload["ik_clip_count"]
        ik_clip_worst_mm = payload["ik_clip_worst_mm"]
        joint_clip_count = payload["joint_clip_count"]
        joint_clip_worst_deg = payload["joint_clip_worst_deg"]
        stale_drop_count = payload["stale_drop_count"]
        malformed_drop_count = payload["malformed_drop_count"]
    except KeyError as exc:
        raise ProtocolError(f"telemetry missing field {exc}") from None

    if not isinstance(ok, bool):
        raise ProtocolError(f"ok must be a bool, got {ok!r}")
    if not isinstance(fault_flags, int) or isinstance(fault_flags, bool) or fault_flags < 0:
        raise ProtocolError(f"invalid fault_flags: {fault_flags!r}")
    if not isinstance(calibration_armed, bool):
        raise ProtocolError(f"calibration_armed must be a bool, got {calibration_armed!r}")
    if not isinstance(bench_armed, bool):
        raise ProtocolError(f"bench_armed must be a bool, got {bench_armed!r}")
    if not isinstance(robot_assembled, bool):
        raise ProtocolError(f"robot_assembled must be a bool, got {robot_assembled!r}")
    if not isinstance(link_timeout_s, (int, float)) or isinstance(link_timeout_s, bool):
        raise ProtocolError(f"link_timeout_s must be a number, got {link_timeout_s!r}")
    if not isinstance(ik_clip_count, int) or isinstance(ik_clip_count, bool) or ik_clip_count < 0:
        raise ProtocolError(f"invalid ik_clip_count: {ik_clip_count!r}")
    if not isinstance(ik_clip_worst_mm, (int, float)) or isinstance(ik_clip_worst_mm, bool) or ik_clip_worst_mm < 0:
        raise ProtocolError(f"invalid ik_clip_worst_mm: {ik_clip_worst_mm!r}")
    if not isinstance(joint_clip_count, int) or isinstance(joint_clip_count, bool) or joint_clip_count < 0:
        raise ProtocolError(f"invalid joint_clip_count: {joint_clip_count!r}")
    if (
        not isinstance(joint_clip_worst_deg, (int, float))
        or isinstance(joint_clip_worst_deg, bool)
        or joint_clip_worst_deg < 0
    ):
        raise ProtocolError(f"invalid joint_clip_worst_deg: {joint_clip_worst_deg!r}")

    for name, value in (("stale_drop_count", stale_drop_count), ("malformed_drop_count", malformed_drop_count)):
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ProtocolError(f"invalid {name}: {value!r}")

    last_accepted_seq = payload.get("last_accepted_seq")
    if last_accepted_seq is not None:
        last_accepted_seq = _require_seq(last_accepted_seq)

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

    raw_profiles = payload.get("profiles")
    profiles = None
    if raw_profiles is not None:
        if not isinstance(raw_profiles, list):
            raise ProtocolError("profiles must be a list or null")
        profiles = tuple(ServoProfile.from_dict(p) for p in raw_profiles)

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
        bench_armed=bench_armed,
        robot_assembled=robot_assembled,
        ik_clip_count=ik_clip_count,
        ik_clip_worst_mm=float(ik_clip_worst_mm),
        joint_clip_count=joint_clip_count,
        joint_clip_worst_deg=float(joint_clip_worst_deg),
        stale_drop_count=stale_drop_count,
        malformed_drop_count=malformed_drop_count,
        last_accepted_seq=last_accepted_seq,
        last_applied=last_applied,
        profiles=profiles,
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
