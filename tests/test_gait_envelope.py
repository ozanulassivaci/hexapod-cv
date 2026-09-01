"""Confirms robot/gait.py's GAIT_ENVELOPE_* constants (the first of the
three limit tiers -- see robot/safe_limit_check.py) haven't gone stale.

Those constants were derived from a slow, full-resolution single-leg
sweep (1,095,000 samples, ~6s -- see their own comment for the exact
grid and how to regenerate them) and then hardcoded, the same "manual
codegen, not a hook" pattern as the golden fixtures (docs/protocol.md
Section 5) -- forgettable by construction. This test re-sweeps at a much
coarser resolution (fast enough for every test run) and checks the
hardcoded constants against it with a generous tolerance: not tight
enough to replace the full derivation, but tight enough that changing
STEP_HEIGHT_MM, STEP_LENGTH_MM, BODY_HEIGHT_Z_CROUCHED_MM/TALL_MM, or
ROTATION_STEP_RAD without regenerating the constants fails loudly here
instead of silently invalidating robot/safe_limit_check.py's fit check.
"""

import itertools
import math

import pytest

from robot.gait import (
    GAIT_ENVELOPE_COXA_MAX_DEG,
    GAIT_ENVELOPE_COXA_MIN_DEG,
    GAIT_ENVELOPE_FEMUR_MAX_DEG,
    GAIT_ENVELOPE_FEMUR_MIN_DEG,
    GAIT_ENVELOPE_TIBIA_MAX_DEG,
    GAIT_ENVELOPE_TIBIA_MIN_DEG,
    foot_target,
)
from robot.kinematics import LEGS, clamp_joint_angles, inverse_kinematics, to_servo_deg

# Coarse on purpose -- leg-invariance is already proven elsewhere
# (test_kinematics.py's test_rotational_sense_symmetry_across_all_legs,
# and this sweep's own single-leg-vs-full-sweep cross-check when these
# constants were first derived), so only leg 0 is swept here.
_PHASE_POINTS = 40
_DIRECTION_POINTS = 16
_MAGNITUDES = (0.0, 1.0)
_SPEEDS = (0.0, 50.0, 100.0)
_ROTATIONS = (-1.0, 0.0, 1.0)
_BODY_HEIGHTS = (0.0, 50.0, 100.0)

# Loose: a coarse grid can miss the true extremum by a fraction of a
# degree even when nothing has changed. Wide enough to comfortably absorb
# that, narrow enough that swapping in a materially different constant
# (e.g. the old STEP_HEIGHT_MM=100) fails immediately.
_TOLERANCE_DEG = 1.0


def _sweep_leg0_envelope() -> dict:
    """Bounds in degrees-from-neutral (to_servo_deg()'s convention, see
    robot/gait.py's GAIT_ENVELOPE_* comment) -- coxa/femur match this
    module's raw kinematic angle directly, tibia does not (to_servo_deg()
    negates it), so this reads the already-converted servo-write angles
    rather than JointAngles directly."""
    leg = LEGS[0]
    directions = [2 * math.pi * i / _DIRECTION_POINTS for i in range(_DIRECTION_POINTS)]
    phases = [i / _PHASE_POINTS for i in range(_PHASE_POINTS)]
    bounds = {
        "coxa": [float("inf"), float("-inf")],
        "femur": [float("inf"), float("-inf")],
        "tibia": [float("inf"), float("-inf")],
    }
    for body_height, speed, rotation, mag, dir_rad, phase in itertools.product(
        _BODY_HEIGHTS, _SPEEDS, _ROTATIONS, _MAGNITUDES, directions, phases
    ):
        if mag == 0.0 and dir_rad != 0.0:
            continue
        vx, vy = mag * math.cos(dir_rad), mag * math.sin(dir_rad)
        target = foot_target(0, leg, phase, vx, vy, speed, rotation, body_height)
        clamped = clamp_joint_angles(inverse_kinematics(target, leg))
        coxa_servo, femur_servo, tibia_servo = to_servo_deg(clamped)
        # Coxa/femur's degFromNeutral = servoDeg - 90 (neutralServoDeg);
        # tibia's = tibiaServo - 0. Subtracting neutralServoDeg here
        # rather than comparing raw servoDeg keeps this test's numbers
        # directly comparable to GAIT_ENVELOPE_*_DEG without the reader
        # needing to separately know each joint's neutral by heart.
        for joint, deg in (("coxa", coxa_servo - 90.0), ("femur", femur_servo - 90.0), ("tibia", tibia_servo)):
            bounds[joint][0] = min(bounds[joint][0], deg)
            bounds[joint][1] = max(bounds[joint][1], deg)
    return bounds


def test_gait_envelope_constants_match_a_live_sweep():
    bounds = _sweep_leg0_envelope()

    assert bounds["coxa"][0] == pytest.approx(GAIT_ENVELOPE_COXA_MIN_DEG, abs=_TOLERANCE_DEG)
    assert bounds["coxa"][1] == pytest.approx(GAIT_ENVELOPE_COXA_MAX_DEG, abs=_TOLERANCE_DEG)
    assert bounds["femur"][0] == pytest.approx(GAIT_ENVELOPE_FEMUR_MIN_DEG, abs=_TOLERANCE_DEG)
    assert bounds["femur"][1] == pytest.approx(GAIT_ENVELOPE_FEMUR_MAX_DEG, abs=_TOLERANCE_DEG)
    assert bounds["tibia"][0] == pytest.approx(GAIT_ENVELOPE_TIBIA_MIN_DEG, abs=_TOLERANCE_DEG)
    assert bounds["tibia"][1] == pytest.approx(GAIT_ENVELOPE_TIBIA_MAX_DEG, abs=_TOLERANCE_DEG)


def test_gait_envelope_constants_are_not_tighter_than_the_live_sweep():
    """Separate from the approx check above: a rounded-outward constant
    must never be tighter than what's actually reachable, in either
    direction -- that would make robot/safe_limit_check.py's fit check
    falsely reassuring (a joint that gait can actually drive past the
    stored envelope wouldn't be flagged)."""
    bounds = _sweep_leg0_envelope()

    assert GAIT_ENVELOPE_COXA_MIN_DEG <= bounds["coxa"][0] + 1e-6
    assert GAIT_ENVELOPE_COXA_MAX_DEG >= bounds["coxa"][1] - 1e-6
    assert GAIT_ENVELOPE_FEMUR_MIN_DEG <= bounds["femur"][0] + 1e-6
    assert GAIT_ENVELOPE_FEMUR_MAX_DEG >= bounds["femur"][1] - 1e-6
    assert GAIT_ENVELOPE_TIBIA_MIN_DEG <= bounds["tibia"][0] + 1e-6
    assert GAIT_ENVELOPE_TIBIA_MAX_DEG >= bounds["tibia"][1] - 1e-6
