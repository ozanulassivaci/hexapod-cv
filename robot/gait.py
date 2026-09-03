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
    clamp_joint_angles_with_clip,
    forward_kinematics,
    inverse_kinematics_with_clip,
)

STEP_LENGTH_MM = 60.0

# STEP_HEIGHT_MM=100 was ported directly from the reference's STEP_HEIGHT
# constant (Hexapod_Arduino.ino line 81) -- a real, measured bug in the
# reference, not a porting error. Traced exactly: step_z =
# sin(phase_norm*pi)*STEP_HEIGHT, added directly to home.z with no other
# scaling anywhere in the chain; the reference's own exponential position
# smoothing (WALK_SMOOTHING_FACTOR=0.4) barely attenuates it (98.4% of
# nominal survives in steady state, since the swing half-cycle is much
# slower than the filter's settling time) and this port's slew-rate
# limiting attenuates it not at all (verified: the achieved peak exactly
# equals the nominal constant). At STEP_HEIGHT=100, real peak foot lift
# is ~98-100mm -- enough to put the foot above the coxa mounting plane
# at any of this project's sane standing heights. 20mm: comfortably
# inside "typical hexapod swing height (20-40mm)", and leaves ~10mm of
# clearance below the coxa plane even at the shallowest configured
# stance (BODY_HEIGHT_Z_CROUCHED_MM below) during a full-speed walk --
# the worst case, since step_z doesn't depend on speed/direction/rotation,
# only phase.
STEP_HEIGHT_MM = 20.0

ROTATION_STEP_RAD = math.radians(15.0)

# Cycles/second at speed=100 (speed_frac=1.0) -- matches the reference's
# t += 0.015 per 10ms tick (1.5 Hz), the only cadence Hexapod_Arduino.ino
# was ever validated at. Speed scales this down toward 0, replacing the
# reference's bang-bang direction/rotation with continuous control.
BASE_PHASE_RATE_HZ = 1.5

# Body height (0..100, WalkCommand's runtime command) linearly maps onto
# this home.z range. Chosen (not the reference's compile-time DEFAULT_Z）
# by sweeping D (leg extension) and joint-torque-sensitivity across the
# full theoretical range for this leg's home footprint: D_min (fully
# folded) turns out unreachable at any height here (home l_forward alone
# already exceeds it), so the only real constraint is D_max (fully
# extended, a kinematic singularity) -- the old range (-20/-150) sat at
# 93% of max reach standing still and let gait motion push it over
# entirely (0.06% of a full envelope sweep clipped, up to 8mm over
# reach). This range keeps the worst case at 233.5mm of a 246.75mm max
# (13.2mm / 5.4% margin, zero clipping across the same sweep) and is
# centered near the best-conditioned point (tibia raw angle ~ -90 deg,
# where the femur/tibia interior angle is closest to a right angle --
# coincidentally almost exactly the reference's own DEFAULT_Z=-40mm).
# Costs 40mm of maximum standing height and 10mm of minimum crouch;
# walking dynamics (stride/strafe/turn -- coxa's whole envelope) are
# untouched, since none of it depends on body height.
BODY_HEIGHT_Z_CROUCHED_MM = -30.0  # height=0
BODY_HEIGHT_Z_TALL_MM = -110.0  # height=100

# Per-joint slew-rate bound, degrees/second. Placeholder set comfortably
# under a typical MG996R's own unloaded slew rate (~350 deg/s) so it
# bounds a startup/mode-transition jump without visibly distorting normal
# gait tracking at max speed -- retune once real servos are on the bench
# under load.
MAX_SLEW_DEG_PER_S = 300.0

NEUTRAL_STANCE_ANGLES = JointAngles(coxa_deg=0.0, femur_deg=45.0, tibia_deg=-90.0)

DEFAULT_BODY_HEIGHT = 50.0

# --- Gait envelope (first of three limit tiers -- see robot/safe_limit_check.py) --
#
# What gait's own math ever actually asks a joint to reach, across the
# full commandable space (every phase, direction, speed, rotation, body
# height) with the constants above -- informational only, never itself a
# constraint on anything. This is the tier a mechanical limit (measured on
# the single test leg, reference/ANALYSIS.md's bring-up plan -- stored as
# transport.protocol.ServoProfile.min/max_deg_from_neutral) must fit
# inside, with SAFE_LIMIT_MARGIN_DEG of room to spare, before it's safe to
# let gait actually drive that joint. Firmware never reads these
# constants -- GatedServoDriver enforces the *safe* limit (mechanical
# minus margin) directly, at runtime, on every commanded pulse; this tier
# exists for the design-time question "will that safe limit actually be
# wide enough for what gait needs," checked once real mechanical data
# exists (robot/safe_limit_check.py), not for any runtime path.
#
# Derived from a single-leg sweep (all 6 legs proven leg-invariant --
# ANALYSIS.md Section 3/4) across 100 phase points x 36 directions x
# {0, 0.5, 1.0} magnitude x 5 speeds x 5 rotations x 6 body heights
# (1,095,000 samples, ~6s). Each bound rounded outward (away from zero
# padding, i.e. min rounded down / max rounded up) to 3 decimal places so
# the stored constant is never tighter than what was actually swept.
# Regenerate by re-running that sweep (same shape as
# tests/test_gait_envelope.py's own coarser live check, at full
# resolution) whenever STEP_HEIGHT_MM, STEP_LENGTH_MM,
# BODY_HEIGHT_Z_CROUCHED_MM/TALL_MM, or ROTATION_STEP_RAD change -- a
# stale envelope here silently invalidates the "does the envelope fit
# inside the safe limit" check without any test catching it, which is
# exactly why that coarser live check exists.
#
# In degrees from this joint's own neutral (to_servo_deg()'s convention,
# the same one transport.protocol.ServoProfile.min/max_deg_from_neutral
# is stored in -- these constants exist specifically to be compared
# against that field, see robot/safe_limit_check.py), NOT this module's
# raw kinematic angle convention. Coxa/femur are identical between the
# two conventions (to_servo_deg() only adds a constant 90, which cancels
# against neutralServoDeg=90), so their bounds are the swept
# JointAngles.coxa_deg/femur_deg extrema directly. Tibia is not: raw
# tibia_deg is provably <= 0 (ANALYSIS.md Section 2's acos-range proof)
# while to_servo_deg() negates it (tibiaServo = -tibiaDeg, neutral 0), so
# tibia's degFromNeutral envelope is the *negation* of the swept raw
# extrema, with min/max swapping accordingly -- swept raw tibia_deg
# was [-114.372, -39.583]; negated and reordered, that's this pair.
GAIT_ENVELOPE_COXA_MIN_DEG = -19.614
GAIT_ENVELOPE_COXA_MAX_DEG = 19.615
GAIT_ENVELOPE_FEMUR_MIN_DEG = -2.130
GAIT_ENVELOPE_FEMUR_MAX_DEG = 70.473
GAIT_ENVELOPE_TIBIA_MIN_DEG = 39.583
GAIT_ENVELOPE_TIBIA_MAX_DEG = 114.372


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


def _ik_goal(target: Point3, leg: Leg) -> tuple[JointAngles, float, float]:
    """One leg's IK goal plus both clip mechanisms' overshoot for this
    call (0.0 for either that didn't fire) -- the shared step used by
    both GaitState.initial() and step(), so the two never drift apart on
    how clip stats are computed."""
    raw_angles, d_overshoot_mm = inverse_kinematics_with_clip(target, leg)
    clamped_angles, joint_overshoot_deg = clamp_joint_angles_with_clip(raw_angles)
    return clamped_angles, d_overshoot_mm, joint_overshoot_deg


@dataclass(frozen=True)
class GaitState:
    """phase plus every leg's currently-tracked (slew-limited) joint
    angles -- the only mutable-in-spirit state in this module; step()
    returns a new GaitState rather than mutating, matching the frozen-
    dataclass style used throughout this codebase's pure-logic modules.

    ik_clip_count/ik_clip_worst_mm and joint_clip_count/
    joint_clip_worst_deg are cumulative since GaitState.initial() (never
    reset by step()) -- ANALYSIS.md Section 5.7 flagged both of IK's
    clip mechanisms (D clamped into the reachable annulus;  each joint
    angle clamped to its configured bound) as silent. Each is now a real
    per-leg-per-tick event count plus the worst overshoot ever seen, so a
    fault buried in this counter is visible instead of invisibly
    discarded. clipped_this_tick reflects only the most recent step()
    call (or initial() itself) -- the level-triggered signal callers use
    to derive an operator-facing fault bit, as opposed to the cumulative
    counters, which are the diagnostic detail behind it. Given the
    verified-safe gait envelope (zero clip events across the full
    theoretical command space, see robot/gait.py's BODY_HEIGHT_Z_*/
    STEP_HEIGHT_MM derivation), any clip firing during normal gait today
    is genuinely anomalous, not routine -- worth surfacing, not just
    counting quietly. That won't stay true once Test Leg exploration mode
    (deliberately probing past limits) exists; whichever code adds that
    mode needs to stop treating its clips as anomalous too."""

    phase: float
    body_height: float
    leg_angles: tuple[JointAngles, ...]
    ik_clip_count: int = 0
    ik_clip_worst_mm: float = 0.0
    joint_clip_count: int = 0
    joint_clip_worst_deg: float = 0.0
    clipped_this_tick: bool = False

    @classmethod
    def initial(cls, body_height: float = DEFAULT_BODY_HEIGHT) -> "GaitState":
        angles = []
        ik_clip_count = joint_clip_count = 0
        ik_clip_worst_mm = joint_clip_worst_deg = 0.0
        for leg in LEGS:
            clamped_angles, d_overshoot_mm, joint_overshoot_deg = _ik_goal(home_position(leg, body_height), leg)
            angles.append(clamped_angles)
            if d_overshoot_mm > 0.0:
                ik_clip_count += 1
                ik_clip_worst_mm = max(ik_clip_worst_mm, d_overshoot_mm)
            if joint_overshoot_deg > 0.0:
                joint_clip_count += 1
                joint_clip_worst_deg = max(joint_clip_worst_deg, joint_overshoot_deg)
        return cls(
            phase=0.0,
            body_height=body_height,
            leg_angles=tuple(angles),
            ik_clip_count=ik_clip_count,
            ik_clip_worst_mm=ik_clip_worst_mm,
            joint_clip_count=joint_clip_count,
            joint_clip_worst_deg=joint_clip_worst_deg,
            clipped_this_tick=(ik_clip_count > 0 or joint_clip_count > 0),
        )

    def foot_positions(self) -> tuple[Point3, ...]:
        return tuple(forward_kinematics(angles, leg) for angles, leg in zip(self.leg_angles, LEGS))


def _slew_one(current: float, goal: float, max_delta: float) -> float:
    delta = goal - current
    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta
    return current + delta


def slew_limit(current: JointAngles, goal: JointAngles, max_delta_deg: float) -> JointAngles:
    """One tick of the per-joint velocity bound step() applies. Public
    because control/gait_preview.py drives a single leg through the same
    math with phase under manual control, and reimplementing this there
    would mean the preview no longer bounded motion the way real gait
    does -- the one property that makes it safe to point at hardware."""
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
    ik_clip_count = state.ik_clip_count
    ik_clip_worst_mm = state.ik_clip_worst_mm
    joint_clip_count = state.joint_clip_count
    joint_clip_worst_deg = state.joint_clip_worst_deg
    clipped_this_tick = False
    for i, leg in enumerate(LEGS):
        goal_point = foot_target(i, leg, new_phase, vx, vy, speed, rotation, body_height)
        goal_angles, d_overshoot_mm, joint_overshoot_deg = _ik_goal(goal_point, leg)
        if d_overshoot_mm > 0.0:
            ik_clip_count += 1
            ik_clip_worst_mm = max(ik_clip_worst_mm, d_overshoot_mm)
            clipped_this_tick = True
        if joint_overshoot_deg > 0.0:
            joint_clip_count += 1
            joint_clip_worst_deg = max(joint_clip_worst_deg, joint_overshoot_deg)
            clipped_this_tick = True
        new_angles.append(slew_limit(state.leg_angles[i], goal_angles, max_delta_deg))

    return GaitState(
        phase=new_phase,
        body_height=body_height,
        leg_angles=tuple(new_angles),
        ik_clip_count=ik_clip_count,
        ik_clip_worst_mm=ik_clip_worst_mm,
        joint_clip_count=joint_clip_count,
        joint_clip_worst_deg=joint_clip_worst_deg,
        clipped_this_tick=clipped_this_tick,
    )
