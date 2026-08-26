"""Human-readable name for each servo_index (0..17), for the calibration
tab so the operator isn't counting in their head at 2am.

There is no real SERVO_MAP yet -- nothing is assembled (CLAUDE.md). This
assumes servo_index = leg_index * 3 + joint_index, legs ordered
[RF, RM, RR, LR, LM, LF] (matching the reference firmware's array order,
confirmed consecutive-around-the-ring in ANALYSIS.md Section 4), joints
ordered [coxa, femur, tibia] per leg. This is a documented placeholder for
UI display, not a wiring decision -- update it once a real SERVO_MAP exists
and servo_index is actually assigned to physical channels.
"""

from transport.protocol import SERVO_COUNT

_LEG_NAMES = ["RF", "RM", "RR", "LR", "LM", "LF"]
_JOINT_NAMES = ["coxa", "femur", "tibia"]


def servo_name(servo_index: int) -> str:
    if not (0 <= servo_index < SERVO_COUNT):
        raise ValueError(f"servo_index out of range: {servo_index!r}")
    leg_index, joint_index = divmod(servo_index, len(_JOINT_NAMES))
    return f"{_LEG_NAMES[leg_index]} {_JOINT_NAMES[joint_index]}"


SERVO_NAMES = [servo_name(i) for i in range(SERVO_COUNT)]
