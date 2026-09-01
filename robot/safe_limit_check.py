"""Design-time question, not a runtime path: does the gait envelope
(robot/gait.py's GAIT_ENVELOPE_* -- what gait's own math ever actually
asks a joint to reach) fit inside a servo's *safe* limit (its
bench-recorded mechanical limit, transport.protocol.ServoProfile's
min/max_deg_from_neutral, shrunk inward by SAFE_LIMIT_MARGIN_DEG) with
room to spare?

Firmware enforces the safe limit directly, on every commanded pulse,
independent of whether anyone ever ran this check
(firmware/lib/core/GatedServoDriver.h's LimitMode::Safe) -- a joint whose
envelope doesn't fit isn't unsafe because this check wasn't run, it's
unsafe because gait will get pulses refused (main.cpp's
FAULT_SERVO_FAULT_BIT) exactly when it tries to reach the part of its
own envelope that doesn't fit. This is the offline "will that actually
have enough room" question, meant to be run once a Test Leg session has
recorded real mechanical data (reference/ANALYSIS.md's bring-up plan) --
there is nothing to check against yet, since nothing is assembled
(CLAUDE.md).
"""

from dataclasses import dataclass

from robot.gait import (
    GAIT_ENVELOPE_COXA_MAX_DEG,
    GAIT_ENVELOPE_COXA_MIN_DEG,
    GAIT_ENVELOPE_FEMUR_MAX_DEG,
    GAIT_ENVELOPE_FEMUR_MIN_DEG,
    GAIT_ENVELOPE_TIBIA_MAX_DEG,
    GAIT_ENVELOPE_TIBIA_MIN_DEG,
)
from robot.kinematics import SAFE_LIMIT_MARGIN_DEG

_JOINT_NAMES = ("coxa", "femur", "tibia")

_JOINT_ENVELOPE_DEG = {
    "coxa": (GAIT_ENVELOPE_COXA_MIN_DEG, GAIT_ENVELOPE_COXA_MAX_DEG),
    "femur": (GAIT_ENVELOPE_FEMUR_MIN_DEG, GAIT_ENVELOPE_FEMUR_MAX_DEG),
    "tibia": (GAIT_ENVELOPE_TIBIA_MIN_DEG, GAIT_ENVELOPE_TIBIA_MAX_DEG),
}


def joint_for_servo(servo_index: int) -> str:
    """coxa/femur/tibia = servo_index % 3 -- matches
    transport.protocol.neutral_pulse_us_for_servo and
    ui/servo_names.py's documented placeholder convention, pending a
    real SERVO_MAP (CLAUDE.md)."""
    return _JOINT_NAMES[servo_index % 3]


@dataclass(frozen=True)
class FitResult:
    servo_index: int
    joint: str
    fits: bool
    # How much room is left, in degrees, between the safe limit and what
    # gait's envelope actually needs on that end -- positive means spare
    # room, negative means the envelope reaches past the safe limit by
    # that much (gait would get pulses refused there in practice). None
    # on whichever end has no recorded mechanical limit at all yet:
    # "unmeasured" is "unknown", not "unlimited", and must not be
    # silently scored as a pass.
    low_spare_deg: float | None
    high_spare_deg: float | None


def check_fit(servo_index: int, min_deg_from_neutral: float | None, max_deg_from_neutral: float | None) -> FitResult:
    """min/max_deg_from_neutral: this servo's own recorded MECHANICAL
    limit (transport.protocol.ServoProfile's own fields, same None
    convention for "not bench-tested yet"). Compares against
    SAFE_LIMIT_MARGIN_DEG inward of that, not the raw mechanical bound --
    the same shrinking firmware's LimitMode::Safe applies at runtime."""
    joint = joint_for_servo(servo_index)
    env_min_deg, env_max_deg = _JOINT_ENVELOPE_DEG[joint]

    low_spare_deg = None
    if min_deg_from_neutral is not None:
        safe_min_deg = min_deg_from_neutral + SAFE_LIMIT_MARGIN_DEG
        low_spare_deg = env_min_deg - safe_min_deg

    high_spare_deg = None
    if max_deg_from_neutral is not None:
        safe_max_deg = max_deg_from_neutral - SAFE_LIMIT_MARGIN_DEG
        high_spare_deg = safe_max_deg - env_max_deg

    fits = low_spare_deg is not None and low_spare_deg >= 0.0 and high_spare_deg is not None and high_spare_deg >= 0.0
    return FitResult(
        servo_index=servo_index,
        joint=joint,
        fits=fits,
        low_spare_deg=low_spare_deg,
        high_spare_deg=high_spare_deg,
    )


def check_all(profiles) -> tuple[FitResult, ...]:
    """profiles: anything indexable by servo_index (0-17) yielding an
    object with .min_deg_from_neutral/.max_deg_from_neutral --
    transport.protocol.ServoProfile itself, or the tuple
    Telemetry.profiles already carries after a read_offsets round trip."""
    return tuple(
        check_fit(i, profiles[i].min_deg_from_neutral, profiles[i].max_deg_from_neutral) for i in range(18)
    )
