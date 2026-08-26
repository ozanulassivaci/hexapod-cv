"""Pure tripod gait engine: phase advance, per-leg foot-target
generation, slew-rate-limited joint tracking. No sockets, no Qt -- the
Python reference implementation firmware/lib/core/Gait.cpp is ported
from and checked against (tests/fixtures/gait_golden.json).

Ported from Hexapod_Arduino.ino's get_foot_target/set_foot_positions,
closing the gaps reference/ANALYSIS.md Section 4 identifies:
- continuous speed scaling (0..100), not bang-bang {-1,0,+1}
- strafing (vy), not just a single forward axis
- runtime body height, not a compile-time constant
- a real per-joint slew-rate bound (degrees/second), replacing the
  reference's position-only exponential smoothing -- ANALYSIS.md safety
  gap #3 ("position smoothing is not a velocity limit... nothing bounds
  servo slew rate directly"). Bounding in joint-angle space (not foot-
  position space) is deliberate: IK is nonlinear, so a bounded mm/tick
  foot movement does not itself bound deg/tick joint movement near a
  leg's singularities. Bounding after IK is what actually bounds servo
  slew rate, which is the literal thing safety gap #3 asks for.
"""

import math
from dataclasses import dataclass

from robot.kinematics import (
    LEGS,
    Leg,
    JointAngles,
    Point3,
    clamp_joint_angles,
    forward_kinematics,
    inverse_kinematics,
)

STEP_LENGTH_MM = 60.0
STEP_HEIGHT_MM = 100.0
ROTATION_STEP_RAD = math.radians(15.0)

# Cycles/second at speed=100 (speed_frac=1.0) -- matches the reference's
# t += 0.015 per 10ms tick (1.5 Hz), the only cadence Hexapod_Arduino.ino
# was ever validated at. Speed scales this down toward 0, replacing the
# reference's bang-bang direction/rotation with continuous control.
BASE_PHASE_RATE_HZ = 1.5

# Body height (0..100, WalkCommand's runtime command) linearly maps onto
# this home.z range. DEFAULT_Z in the reference was a compile-time -40mm;
# these bounds are placeholders bracketing it, not measured against an
# assembled frame -- narrow once real reachability/ground-clearance data
# exists, same caveat as the joint angle clamps in kinematics.py.
BODY_HEIGHT_Z_CROUCHED_MM = -20.0  # height=0
BODY_HEIGHT_Z_TALL_MM = -150.0  # height=100

# Per-joint slew-rate bound, degrees/second. Placeholder set comfortably
# under a typical MG996R's own unloaded slew rate (~350 deg/s) so it
# bounds a startup/mode-transition jump without visibly distorting normal
# gait tracking at max speed -- retune once real servos are on the bench
# under load.
MAX_SLEW_DEG_PER_S = 300.0

NEUTRAL_STANCE_ANGLES = JointAngles(coxa_deg=0.0, femur_deg=45.0, tibia_deg=-90.0)

DEFAULT_BODY_HEIGHT = 50.0


def home_xy_mm(leg: Leg) -> tuple[float, float]:
    """Body-frame (x, y) of this leg's neutral stance foot position --
    mirrors setup()'s calculate_fk(neutralAngles, ...) call."""
    point = forward_kinematics(NEUTRAL_STANCE_ANGLES, leg)
    return point.x, point.y


def home_z_mm(body_height: float) -> float:
    frac = max(0.0, min(100.0, body_height)) / 100.0
    return BODY_HEIGHT_Z_CROUCHED_MM + frac * (BODY_HEIGHT_Z_TALL_MM - BODY_HEIGHT_Z_CROUCHED_MM)


def home_position(leg: Leg, body_height: float) -> Point3:
    x, y = home_xy_mm(leg)
    return Point3(x, y, home_z_mm(body_height))


def is_in_stance(leg_index: int, phase: float) -> bool:
    """True if this leg is in its foot-planted half of the tripod cycle
    at the given global phase -- local_phase < 0.5, matching
    get_foot_target's phase<0.5 branch (step_z=0, foot at ground level)
    vs. phase>=0.5 (foot lifted). Tripod grouping by leg_index % 2 --
    ANALYSIS.md Section 4's confirmed-correct grouping."""
    local_phase = phase % 1.0
    if leg_index % 2 != 0:
        local_phase = (local_phase + 0.5) % 1.0
    return local_phase < 0.5


def is_motion_idle(vx: float, vy: float, speed: float, rotation: float) -> bool:
    """The idle condition foot_target() short-circuits on -- exposed so
    firmware's bench-mode arm-refusal check ("refuse to arm bench_mode
    while gait is actively non-idle") and this module share one
    definition of "idle" (mirrored in firmware/lib/core/Gait.cpp's
    isMotionIdle for the same reason)."""
    speed_frac = max(0.0, min(100.0, speed)) / 100.0
    if speed_frac <= 1e-9:
        return True
    magnitude = math.hypot(vx, vy)
    return magnitude <= 1e-9 and abs(rotation) <= 1e-9


def foot_target(
    leg_index: int,
    leg: Leg,
    phase: float,
    vx: float,
    vy: float,
    speed: float,
    rotation: float,
    body_height: float,
) -> Point3:
    """Mirrors get_foot_target, generalized. vx, vy in [-1, 1] (jointly
    magnitude-clamped to 1 here, so an unnormalized diagonal command
    can't exceed STEP_LENGTH_MM -- the GUI already normalizes its own
    WASD intent, but this must hold for any client). speed in [0, 100]
    (WalkCommand/TurnCommand's own field). rotation in [-1, 1]."""
    home = home_position(leg, body_height)

    if is_motion_idle(vx, vy, speed, rotation):
        return home

    speed_frac = max(0.0, min(100.0, speed)) / 100.0
    magnitude = math.hypot(vx, vy)
    if magnitude > 1.0:
        vx, vy = vx / magnitude, vy / magnitude

    local_phase = phase % 1.0
    if leg_index % 2 != 0:
        local_phase = (local_phase + 0.5) % 1.0

    if local_phase < 0.5:
        phase_norm = local_phase * 2.0
        step_scale = 0.5 - phase_norm
        step_z = 0.0
    else:
        phase_norm = (local_phase - 0.5) * 2.0
        step_scale = phase_norm - 0.5
        step_z = math.sin(phase_norm * math.pi) * STEP_HEIGHT_MM

    step_x = vx * speed_frac * step_scale * STEP_LENGTH_MM
    step_y = vy * speed_frac * step_scale * STEP_LENGTH_MM
    step_rot_rad = rotation * speed_frac * step_scale * ROTATION_STEP_RAD

    c_rot, s_rot = math.cos(step_rot_rad), math.sin(step_rot_rad)
    rotated_x = home.x * c_rot - home.y * s_rot
    rotated_y = home.x * s_rot + home.y * c_rot

    return Point3(rotated_x + step_x, rotated_y + step_y, home.z + step_z)


@dataclass(frozen=True)
class GaitState:
    """phase plus every leg's currently-tracked (slew-limited) joint
    angles -- the only mutable-in-spirit state in this module; step()
    returns a new GaitState rather than mutating, matching the frozen-
    dataclass style used throughout this codebase's pure-logic modules."""

    phase: float
    body_height: float
    leg_angles: tuple[JointAngles, ...]

    @classmethod
    def initial(cls, body_height: float = DEFAULT_BODY_HEIGHT) -> "GaitState":
        angles = tuple(
            clamp_joint_angles(inverse_kinematics(home_position(leg, body_height), leg)) for leg in LEGS
        )
        return cls(phase=0.0, body_height=body_height, leg_angles=angles)

    def foot_positions(self) -> tuple[Point3, ...]:
        return tuple(forward_kinematics(angles, leg) for angles, leg in zip(self.leg_angles, LEGS))


def _slew_one(current: float, goal: float, max_delta: float) -> float:
    delta = goal - current
    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta
    return current + delta


def _slew_limit(current: JointAngles, goal: JointAngles, max_delta_deg: float) -> JointAngles:
    return JointAngles(
        coxa_deg=_slew_one(current.coxa_deg, goal.coxa_deg, max_delta_deg),
        femur_deg=_slew_one(current.femur_deg, goal.femur_deg, max_delta_deg),
        tibia_deg=_slew_one(current.tibia_deg, goal.tibia_deg, max_delta_deg),
    )


def step(
    state: GaitState,
    dt_s: float,
    vx: float,
    vy: float,
    speed: float,
    rotation: float,
    body_height: float,
) -> GaitState:
    """Advances phase by BASE_PHASE_RATE_HZ * speed_frac * dt_s (frozen
    when speed is ~0, so an idle robot's phase never drifts), computes
    each leg's fresh IK goal, and slew-limits every joint toward it by at
    most MAX_SLEW_DEG_PER_S * dt_s -- this single mechanism is what
    replaces the reference's unbounded exponential smoothing, and is
    what makes startup ramp, stop ramp, and ordinary tracking all
    provably bounded (see the worst-case test in tests/test_gait.py).

    dt_s must be real elapsed time -- this has no concept of its own
    clock, same separation as control/tracker.py and control/bench.py."""
    speed_frac = max(0.0, min(100.0, speed)) / 100.0
    new_phase = (state.phase + BASE_PHASE_RATE_HZ * speed_frac * dt_s) % 1.0
    max_delta_deg = MAX_SLEW_DEG_PER_S * max(0.0, dt_s)

    new_angles = []
    for i, leg in enumerate(LEGS):
        goal_point = foot_target(i, leg, new_phase, vx, vy, speed, rotation, body_height)
        goal_angles = clamp_joint_angles(inverse_kinematics(goal_point, leg))
        new_angles.append(_slew_limit(state.leg_angles[i], goal_angles, max_delta_deg))

    return GaitState(phase=new_phase, body_height=body_height, leg_angles=tuple(new_angles))
