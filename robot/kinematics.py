"""Pure FK/IK for one hexapod leg. No sockets, no Qt -- ported from
reference/Hexapod_Arduino.ino's calculate_fk/calculate_ik, per
reference/ANALYSIS.md Sections 1-3 and 7. This is the Python reference
implementation firmware/lib/core/Kinematics.cpp is ported from and
checked against (tests/fixtures/kinematics_golden.json) -- see
ANALYSIS.md Section 7 for why two implementations, not one shared via a
Python extension.

Degrees at the boundary (this module's public functions), radians
internally -- matches the reference's own convention and CLAUDE.md's
kinematics rule. PCA9685 pulse units never appear here; that conversion
is firmware's own single leaf function (AngleToPulse), per ANALYSIS.md
Section 7's "degrees everywhere except one leaf function".
"""

import math
from dataclasses import dataclass

COXA_LENGTH_MM = 38.0
FEMUR_LENGTH_MM = 86.25
TIBIA_LENGTH_MM = 160.5

# Joint angle clamps, in this module's raw (unshifted) angle convention --
# see JointAngles below. Coxa's [-45, 45] is the reference's own
# constrain(90+angle, 45, 135), ported as-is (ANALYSIS.md Section 5.1's
# only proven-safe number). Femur/tibia have no reference precedent (the
# reference never clamped them at all -- the exact gap ANALYSIS.md
# Section 5.1 flags as a stall/burn risk). These are placeholders set to
# a standard hobby servo's own full rated travel (180 degrees), the only
# bound provable without assembled-frame collision data -- narrow once
# real mechanical limits are known, same caveat as firmware's
# PCA9685_OSC_FREQ placeholders. Tibia's range is negative because
# JointAngles.tibia_deg is raw gamma (see below), which is provably <= 0.
COXA_MIN_DEG = -45.0
COXA_MAX_DEG = 45.0
FEMUR_MIN_DEG = -90.0
FEMUR_MAX_DEG = 90.0
TIBIA_MIN_DEG = -180.0
TIBIA_MAX_DEG = 0.0


@dataclass(frozen=True)
class Leg:
    """Kinematic identity only -- mount origin and the mount angle
    derived from it. Hardware addressing (board/channel, calibration)
    lives elsewhere (firmware's ServoMap/ServoProfile), never here --
    ANALYSIS.md Section 7's "per-leg struct vs. flat SERVO_MAP" split."""

    name: str
    origin_x_mm: float
    origin_y_mm: float

    @property
    def mount_angle_rad(self) -> float:
        return math.atan2(self.origin_y_mm, self.origin_x_mm)


# Leg origins from Hexapod_Arduino.ino, in the declared array order (RF,
# RM, RR, LR, LM, LF) -- consecutive around the physical ring, confirmed
# in ANALYSIS.md Section 4 (this order is what makes leg_index % 2 the
# correct tripod grouping in robot/gait.py).
LEGS: tuple[Leg, ...] = (
    Leg("RF", 100.94, -57.11),
    Leg("RM", 0.0, -105.72),
    Leg("RR", -100.94, -57.11),
    Leg("LR", -100.94, 57.11),
    Leg("LM", 0.0, 105.72),
    Leg("LF", 100.94, 57.11),
)


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class JointAngles:
    """Degrees, raw math convention -- the same convention
    calculate_fk/calculate_ik use internally in the reference, and what
    forward_kinematics/inverse_kinematics below consume and return, so
    the two are genuine inverses of each other (needed for the round-trip
    property tests in tests/test_kinematics.py). NOT the servo command
    convention -- call to_servo_deg() to get that. tibia_deg here is raw
    gamma: for every reachable target it's provably <= 0 (ANALYSIS.md
    Section 2's acos-range proof; see the invariant test)."""

    coxa_deg: float
    femur_deg: float
    tibia_deg: float


def to_servo_deg(angles: JointAngles) -> tuple[float, float, float]:
    """The one place the reference's servo-write convention is encoded:
    coxa/femur are bipolar, centered on 90 (matches NEUTRAL_PULSE_US);
    tibia is zero-based and negated (ANALYSIS.md Section 2 -- replaces
    the reference's fabs(angles.z) with an explicit -angles.z, same
    output, fails loudly instead of silently if the >=0 invariant is
    ever broken by a future IK change). Returns (coxa, femur, tibia) in
    degrees, unclamped -- call clamp_joint_angles() on the JointAngles
    first."""
    return (90.0 + angles.coxa_deg, 90.0 + angles.femur_deg, -angles.tibia_deg)


def clamp_joint_angles(angles: JointAngles) -> JointAngles:
    """Every joint clamped, not just coxa -- closes ANALYSIS.md safety
    gap #5.1 ("only the coxa angle is clamped... nothing catches it if
    the formula changes"). Applied after inverse_kinematics(), before a
    result is ever used to command hardware. Discards how far past a
    bound the input was -- call clamp_joint_angles_with_clip() where that
    matters (the live gait loop's telemetry)."""
    clamped, _ = clamp_joint_angles_with_clip(angles)
    return clamped


def clamp_joint_angles_with_clip(angles: JointAngles) -> tuple[JointAngles, float]:
    """Same clamp as clamp_joint_angles(), plus the worst-offending
    joint's overshoot in degrees (0.0 if nothing needed clamping). This
    is what makes gap #5.1's clamping observable instead of silently
    discarded -- robot/gait.py::step feeds it into GaitState's cumulative
    joint_clip_count/joint_clip_worst_deg."""

    def _clamp(value: float, lo: float, hi: float) -> tuple[float, float]:
        if value > hi:
            return hi, value - hi
        if value < lo:
            return lo, lo - value
        return value, 0.0

    coxa_deg, coxa_over = _clamp(angles.coxa_deg, COXA_MIN_DEG, COXA_MAX_DEG)
    femur_deg, femur_over = _clamp(angles.femur_deg, FEMUR_MIN_DEG, FEMUR_MAX_DEG)
    tibia_deg, tibia_over = _clamp(angles.tibia_deg, TIBIA_MIN_DEG, TIBIA_MAX_DEG)
    worst_over_deg = max(coxa_over, femur_over, tibia_over)
    return JointAngles(coxa_deg=coxa_deg, femur_deg=femur_deg, tibia_deg=tibia_deg), worst_over_deg


def forward_kinematics(angles: JointAngles, leg: Leg, origin_z_mm: float = 0.0) -> Point3:
    """Mirrors calculate_fk. origin_z_mm is the leg's body-frame mount
    height (typically 0, all legs mount on the same plane) -- kept as an
    explicit parameter, never packed into a reused field, per
    ANALYSIS.md Section 1's fix for the reference's overloaded
    Vector.z (which doubled as "mount angle in radians" for origin and
    "height in mm" for target/home; this port uses Leg.mount_angle_rad
    and this parameter as two distinct values instead)."""
    alpha = math.radians(angles.coxa_deg) + leg.mount_angle_rad
    beta = math.radians(angles.femur_deg)
    gamma = math.radians(angles.tibia_deg)

    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    cbg, sbg = math.cos(beta + gamma), math.sin(beta + gamma)

    coxa_joint_x = leg.origin_x_mm + COXA_LENGTH_MM * ca
    coxa_joint_y = leg.origin_y_mm + COXA_LENGTH_MM * sa

    foot_x = coxa_joint_x + (FEMUR_LENGTH_MM * cb + TIBIA_LENGTH_MM * cbg) * ca
    foot_y = coxa_joint_y + (FEMUR_LENGTH_MM * cb + TIBIA_LENGTH_MM * cbg) * sa
    foot_z = origin_z_mm + (FEMUR_LENGTH_MM * sb + TIBIA_LENGTH_MM * sbg)

    return Point3(foot_x, foot_y, foot_z)


def inverse_kinematics(target: Point3, leg: Leg, origin_z_mm: float = 0.0) -> JointAngles:
    """Mirrors calculate_ik. See inverse_kinematics_with_clip() for the
    full rationale -- this is a thin wrapper that discards how far past
    d_max/d_min the pre-clamp target was. Returns raw (unclamped) angles
    in the same convention forward_kinematics() consumes -- call
    clamp_joint_angles() and then to_servo_deg() before using the result
    to command hardware."""
    angles, _ = inverse_kinematics_with_clip(target, leg, origin_z_mm)
    return angles


def inverse_kinematics_with_clip(
    target: Point3, leg: Leg, origin_z_mm: float = 0.0
) -> tuple[JointAngles, float]:
    """Same solve as inverse_kinematics(), plus how far past d_max/d_min
    (mm) the pre-clamp D was -- 0.0 if the target was already reachable.
    Rotates the target into the leg's local frame by -mount_angle_rad (a
    pure rotation, never a reflection -- ANALYSIS.md Section 3's proof
    that no per-leg sign flip is needed for any of the six legs), solves
    the femur/tibia planar 2-link problem via law of cosines, clamping D
    into the reachable annulus before acos to avoid a domain error on an
    unreachable target. ANALYSIS.md Section 5.7 notes this clamp used to
    be entirely silent; the overshoot returned here is what makes it
    observable -- robot/gait.py::step feeds it into GaitState's
    cumulative ik_clip_count/ik_clip_worst_mm."""
    relative_x = target.x - leg.origin_x_mm
    relative_y = target.y - leg.origin_y_mm
    relative_z = target.z - origin_z_mm

    c = math.cos(-leg.mount_angle_rad)
    s = math.sin(-leg.mount_angle_rad)
    local_x = relative_x * c - relative_y * s
    local_y = relative_x * s + relative_y * c

    l_xy = math.hypot(local_x, local_y)
    l_forward = l_xy - COXA_LENGTH_MM
    d = math.hypot(l_forward, relative_z)

    d_max = FEMUR_LENGTH_MM + TIBIA_LENGTH_MM
    d_min = abs(FEMUR_LENGTH_MM - TIBIA_LENGTH_MM)
    overshoot_mm = 0.0
    if d > d_max:
        overshoot_mm = d - d_max
        d = d_max - 0.001
    if d < d_min:
        overshoot_mm = d_min - d
        d = d_min + 0.001

    gamma_raw = math.acos(
        (FEMUR_LENGTH_MM**2 + TIBIA_LENGTH_MM**2 - d**2) / (2.0 * FEMUR_LENGTH_MM * TIBIA_LENGTH_MM)
    ) - math.pi  # provably in [-pi, 0] for any clamped d -- see the invariant test

    beta1 = math.atan2(relative_z, l_forward)
    beta2 = math.acos((FEMUR_LENGTH_MM**2 + d**2 - TIBIA_LENGTH_MM**2) / (2.0 * FEMUR_LENGTH_MM * d))
    beta = beta1 + beta2

    coxa_deg = math.degrees(math.atan2(local_y, local_x))
    femur_deg = math.degrees(beta)
    tibia_deg = math.degrees(gamma_raw)  # raw gamma -- see to_servo_deg() for the -gamma servo write

    return JointAngles(coxa_deg=coxa_deg, femur_deg=femur_deg, tibia_deg=tibia_deg), overshoot_mm
