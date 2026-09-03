"""Test Leg gait preview: drives ONE assembled leg through the real gait
cycle, as that leg would move during actual walking. Pure logic, no Qt --
ui/single_leg_tab.py owns the widgets and the transport.

Why this isn't a reimplementation
---------------------------------
Every number here comes from robot/gait.py's own functions, called in the
same order robot.gait.step() calls them per leg:

    foot_target(...) -> inverse_kinematics(...) -> clamp_joint_angles(...)
                     -> slew_limit(...)

step() is not called directly for one reason: it advances phase itself,
from dt and speed, for all six legs at once. The whole point of this mode
is phase under manual control, so phase is the one thing this module owns
and everything else is delegated. tests/test_gait_preview.py pins that
equivalence directly -- it runs a preview tick and robot.gait.step()
side by side and asserts the selected leg's angles match exactly. If
anyone changes the gait math on one side, that test fails rather than the
preview quietly validating a gait the robot doesn't have.

Which leg's geometry
--------------------
The tab's leg-position selector picks a real LEGS[] entry, and this uses
that leg's own origin and mount angle, not a generic one. Two legs given
the same walk command sweep different coxa ranges because they point
different ways on the body -- previewing with the wrong leg's geometry
would show a motion the physical leg is never going to make once it's
bolted to that corner.
"""

from dataclasses import dataclass

from robot.gait import (
    BASE_PHASE_RATE_HZ,
    MAX_SLEW_DEG_PER_S,
    foot_target,
    is_in_stance,
    slew_limit,
)
from robot.kinematics import (
    LEGS,
    JointAngles,
    Leg,
    Point3,
    clamp_joint_angles,
    inverse_kinematics,
)
from robot.safe_limit_check import FitResult, check_fit

# Fraction of real gait speed the preview runs at by default. The first
# time this math drives real hardware it should be slow enough to reach
# RELEASE LEG before anything finishes happening -- 10% turns a ~1.4s
# gait cycle into ~14s, which is walk-pace-to-the-bench slow. Applies to
# both phase advance and the slew-rate bound, so the whole motion scales
# together rather than the leg lurching between slowly-advancing phases.
DEFAULT_SLOW_MOTION = 0.10
MIN_SLOW_MOTION = 0.02
MAX_SLOW_MOTION = 1.0

# Phase step sizes for the step-through control, as a fraction of one
# full cycle.
PHASE_STEPS = (0.01, 0.05, 0.10, 0.25)

# How many points to sample when drawing one cycle's foot path.
TRAJECTORY_SAMPLES = 120


def leg_for_index(leg_index: int) -> Leg:
    return LEGS[leg_index]


def servo_indices_for_leg(leg_index: int) -> tuple[int, int, int]:
    """The three servo_index slots this leg's findings are recorded
    under -- same leg*3 + joint convention the rest of the tab uses."""
    base = leg_index * 3
    return (base, base + 1, base + 2)


# --- pre-flight -----------------------------------------------------------


@dataclass(frozen=True)
class PreflightResult:
    """Whether it's safe to run gait on this leg given what has actually
    been marked. `blocking` is the joints that would have gait drive them
    somewhere unverified -- either past a marked limit, or into a
    direction with no mark at all."""

    fits: bool
    results: tuple[FitResult, ...]

    @property
    def blocking(self) -> tuple[FitResult, ...]:
        return tuple(r for r in self.results if not r.fits)

    def summary_lines(self) -> tuple[str, ...]:
        """One line per problem joint, naming the joint and the gap --
        what the operator needs to decide whether to override, rather
        than a bare refusal."""
        lines = []
        for r in self.blocking:
            if r.low_spare_deg is None and r.high_spare_deg is None:
                lines.append(f"{r.joint}: no mechanical limit marked in either direction")
            elif r.low_spare_deg is None:
                lines.append(f"{r.joint}: no MIN limit marked (MAX has {r.high_spare_deg:+.1f} deg spare)")
            elif r.high_spare_deg is None:
                lines.append(f"{r.joint}: no MAX limit marked (MIN has {r.low_spare_deg:+.1f} deg spare)")
            else:
                short = []
                if r.low_spare_deg < 0.0:
                    short.append(f"MIN short by {abs(r.low_spare_deg):.1f} deg")
                if r.high_spare_deg < 0.0:
                    short.append(f"MAX short by {abs(r.high_spare_deg):.1f} deg")
                lines.append(f"{r.joint}: gait needs more range than marked -- {', '.join(short)}")
        return tuple(lines)


def preflight(leg_index: int, profiles) -> PreflightResult:
    """Does gait's envelope fit inside what's actually been marked for
    this leg's three servos, with SAFE_LIMIT_MARGIN_DEG to spare?

    Deliberately reuses robot/safe_limit_check.py rather than restating
    the comparison: that module already treats an unmarked bound as
    unknown-not-unlimited, which is exactly the partially-marked case
    this gate exists for. profiles is Telemetry.profiles (or anything
    indexable by servo_index); None means nothing has been read back
    yet, which is itself a refusal.
    """
    if profiles is None:
        return PreflightResult(fits=False, results=())
    results = tuple(
        check_fit(i, profiles[i].min_deg_from_neutral, profiles[i].max_deg_from_neutral)
        for i in servo_indices_for_leg(leg_index)
    )
    return PreflightResult(fits=all(r.fits for r in results), results=results)


# --- the preview itself ---------------------------------------------------


def goal_angles(
    leg_index: int,
    phase: float,
    vx: float,
    vy: float,
    speed: float,
    rotation: float,
    body_height: float,
) -> JointAngles:
    """This leg's gait goal at an explicit phase -- robot.gait.step()'s
    per-leg computation, minus the phase advance step() would have done
    itself."""
    leg = leg_for_index(leg_index)
    target = foot_target(leg_index, leg, phase, vx, vy, speed, rotation, body_height)
    return clamp_joint_angles(inverse_kinematics(target, leg))


def advance_phase(phase: float, dt_s: float, speed: float, slow_motion: float) -> float:
    """Live-mode phase advance: robot.gait.step()'s own rate, scaled by
    the preview's slow-motion factor. Frozen when speed is ~0, same as
    step(), so an idle preview's phase never drifts."""
    speed_frac = max(0.0, min(100.0, speed)) / 100.0
    return (phase + BASE_PHASE_RATE_HZ * speed_frac * dt_s * slow_motion) % 1.0


def slew_toward(current: JointAngles, goal: JointAngles, dt_s: float, slow_motion: float) -> JointAngles:
    """One tick of the same per-joint velocity bound real gait applies,
    scaled by slow-motion so step-through and live both approach a new
    pose at a bounded, operator-chosen rate instead of jumping."""
    return slew_limit(current, goal, MAX_SLEW_DEG_PER_S * max(0.0, dt_s) * slow_motion)


def stance_label(leg_index: int, phase: float) -> str:
    """STANCE/SWING at this phase. Worth saying out loud in the UI
    because the bench case inverts the intuition: during stance the foot
    would be planted and carrying the robot, but this leg is clamped with
    its foot in free air, so stance is exactly the part where nothing
    looks like it's happening."""
    return "STANCE" if is_in_stance(leg_index, phase) else "SWING"


@dataclass(frozen=True)
class TrajectoryPoint:
    phase: float
    foot: Point3
    # The joint angles that reach that foot position. Carried so a
    # drawing can plot the path with its own forward-kinematics in the
    # leg's local frame, rather than re-deriving leg-local coordinates
    # from a body-frame point and risking a trace that sits on subtly
    # different axes from the live stick figure beside it.
    angles: JointAngles
    in_stance: bool


def trajectory(
    leg_index: int,
    vx: float,
    vy: float,
    speed: float,
    rotation: float,
    body_height: float,
    samples: int = TRAJECTORY_SAMPLES,
) -> tuple[TrajectoryPoint, ...]:
    """One full cycle of this leg's commanded foot path, for drawing.

    The foot_target() positions themselves, not forward kinematics of
    the slew-limited angles: this is the path gait is *asking* for, which
    is what the operator is judging when they look at the swing arc's
    shape. What the leg actually traces under the slew bound is the live
    marker moving along it."""
    leg = leg_for_index(leg_index)
    points = []
    for i in range(samples):
        phase = i / samples
        target = foot_target(leg_index, leg, phase, vx, vy, speed, rotation, body_height)
        points.append(
            TrajectoryPoint(
                phase=phase,
                foot=target,
                angles=clamp_joint_angles(inverse_kinematics(target, leg)),
                in_stance=is_in_stance(leg_index, phase),
            )
        )
    return tuple(points)
