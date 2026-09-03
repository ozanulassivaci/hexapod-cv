"""The load-bearing test here is
test_preview_goal_matches_the_real_gait_engine_exactly: the whole value of
this mode is that it drives hardware with the gait the robot will actually
walk with, so a preview that quietly diverged from robot/gait.py would
validate nothing while looking like it validated everything.
"""

import math

import pytest

from control import gait_preview as gp
from robot.gait import DEFAULT_BODY_HEIGHT, GaitState, is_in_stance, step
from robot.kinematics import LEGS
from transport.protocol import ServoProfile


def _profiles(min_deg=None, max_deg=None, note=""):
    return [ServoProfile(i, 0, 1, min_deg, max_deg, note) for i in range(18)]


# --- it is the real engine ------------------------------------------------


@pytest.mark.parametrize("leg_index", range(6))
def test_preview_goal_matches_the_real_gait_engine_exactly(leg_index):
    """One preview goal vs one robot.gait.step(), same leg, same phase.

    dt is large enough that step()'s slew limit (300 deg/s * 1s) cannot
    bind on a single tick, so its output is its raw goal -- which is the
    thing being compared. If these ever differ, the preview is showing a
    gait the robot doesn't have.
    """
    vx, vy, speed, rotation = 0.6, -0.3, 80.0, 0.25
    state = GaitState.initial(DEFAULT_BODY_HEIGHT)
    stepped = step(state, 1.0, vx, vy, speed, rotation, DEFAULT_BODY_HEIGHT)

    preview = gp.goal_angles(
        leg_index, stepped.phase, vx, vy, speed, rotation, DEFAULT_BODY_HEIGHT
    )
    engine = stepped.leg_angles[leg_index]

    assert preview.coxa_deg == pytest.approx(engine.coxa_deg, abs=1e-9)
    assert preview.femur_deg == pytest.approx(engine.femur_deg, abs=1e-9)
    assert preview.tibia_deg == pytest.approx(engine.tibia_deg, abs=1e-9)


def test_preview_uses_the_selected_legs_own_geometry():
    # A corner leg and a middle leg do not sweep the same coxa range for
    # the same walk command -- previewing with the wrong one would show a
    # motion that leg never makes. Distinct geometry must produce
    # distinct angles, or the leg selector isn't doing anything.
    a = gp.goal_angles(0, 0.3, 1.0, 0.0, 100.0, 0.0, DEFAULT_BODY_HEIGHT)
    b = gp.goal_angles(1, 0.3, 1.0, 0.0, 100.0, 0.0, DEFAULT_BODY_HEIGHT)
    assert (a.coxa_deg, a.femur_deg, a.tibia_deg) != (b.coxa_deg, b.femur_deg, b.tibia_deg)


def test_leg_for_index_returns_the_real_leg():
    for i in range(6):
        assert gp.leg_for_index(i) is LEGS[i]


# --- phase advance and slew both scale with slow motion -------------------


def test_slow_motion_scales_phase_advance():
    fast = gp.advance_phase(0.0, 0.1, 100.0, 1.0)
    slow = gp.advance_phase(0.0, 0.1, 100.0, 0.1)
    assert slow == pytest.approx(fast * 0.1)


def test_phase_is_frozen_while_speed_is_zero():
    # Same rule step() applies -- an idle preview must not drift.
    assert gp.advance_phase(0.42, 10.0, 0.0, 1.0) == pytest.approx(0.42)


def test_phase_wraps_at_one():
    assert 0.0 <= gp.advance_phase(0.99, 1.0, 100.0, 1.0) < 1.0


def test_slew_toward_is_bounded_and_scaled():
    from robot.kinematics import JointAngles

    current = JointAngles(0.0, 0.0, 0.0)
    goal = JointAngles(90.0, 90.0, -90.0)
    out = gp.slew_toward(current, goal, dt_s=0.05, slow_motion=0.1)
    # 300 deg/s * 0.05s * 0.1 = 1.5 deg per tick, per joint.
    assert out.coxa_deg == pytest.approx(1.5)
    assert out.femur_deg == pytest.approx(1.5)
    assert out.tibia_deg == pytest.approx(-1.5)


def test_slew_toward_never_overshoots_the_goal():
    from robot.kinematics import JointAngles

    current = JointAngles(0.0, 0.0, 0.0)
    goal = JointAngles(0.5, -0.2, 0.1)
    out = gp.slew_toward(current, goal, dt_s=1.0, slow_motion=1.0)
    assert (out.coxa_deg, out.femur_deg, out.tibia_deg) == (0.5, -0.2, 0.1)


# --- stance/swing ---------------------------------------------------------


@pytest.mark.parametrize("leg_index", range(6))
def test_stance_label_agrees_with_the_gait_engine(leg_index):
    for i in range(20):
        phase = i / 20
        expected = "STANCE" if is_in_stance(leg_index, phase) else "SWING"
        assert gp.stance_label(leg_index, phase) == expected


def test_adjacent_legs_are_in_opposite_tripod_halves():
    # The thing that makes previewing "this leg at that position"
    # meaningful at all -- leg_index % 2 grouping, ANALYSIS.md Section 4.
    assert gp.stance_label(0, 0.1) != gp.stance_label(1, 0.1)


# --- trajectory -----------------------------------------------------------


def test_trajectory_covers_one_cycle_split_evenly_between_stance_and_swing():
    points = gp.trajectory(0, 1.0, 0.0, 100.0, 0.0, DEFAULT_BODY_HEIGHT, samples=100)
    assert len(points) == 100
    assert sum(1 for p in points if p.in_stance) == 50
    assert points[0].phase == 0.0
    assert points[-1].phase == pytest.approx(0.99)


def test_trajectory_swing_lifts_the_foot_and_stance_does_not():
    from robot.gait import STEP_HEIGHT_MM

    points = gp.trajectory(0, 1.0, 0.0, 100.0, 0.0, DEFAULT_BODY_HEIGHT)
    stance_z = {round(p.foot.z, 6) for p in points if p.in_stance}
    swing_z = [p.foot.z for p in points if not p.in_stance]
    # Stance is flat: the foot would be planted, so every stance sample
    # sits at exactly one height.
    assert len(stance_z) == 1
    assert max(swing_z) - min(stance_z) == pytest.approx(STEP_HEIGHT_MM, abs=1e-6)


def test_trajectory_is_flat_when_idle():
    # No walk command -> every sample is the home position, so the drawn
    # path collapses to a point rather than showing motion that isn't
    # being commanded.
    points = gp.trajectory(0, 0.0, 0.0, 0.0, 0.0, DEFAULT_BODY_HEIGHT)
    xs = {round(p.foot.x, 9) for p in points}
    zs = {round(p.foot.z, 9) for p in points}
    assert len(xs) == 1 and len(zs) == 1


# --- pre-flight -----------------------------------------------------------


def test_preflight_refuses_when_nothing_is_marked():
    result = gp.preflight(0, _profiles())
    assert result.fits is False
    assert len(result.blocking) == 3
    assert all("no mechanical limit marked" in line for line in result.summary_lines())


def test_preflight_refuses_when_profiles_have_not_been_read():
    result = gp.preflight(0, None)
    assert result.fits is False
    assert result.results == ()


def test_preflight_names_the_joint_and_the_gap():
    # coxa envelope is about +-19.6 deg; marking +-21 leaves only 1.4 deg
    # before the 5 deg safe margin, so it does not fit -- and the message
    # has to say which joint and by how much, not just "no".
    profiles = _profiles()
    profiles[0] = ServoProfile(0, 0, 1, -21.0, 21.0, "")
    result = gp.preflight(0, profiles)
    assert result.fits is False
    coxa_line = next(line for line in result.summary_lines() if line.startswith("coxa"))
    assert "short by" in coxa_line


def test_preflight_passes_when_every_joint_has_room():
    # Generous marks on all three joints of leg 0. Tibia's envelope is
    # 39.6..114.4 deg-from-neutral, so its marks have to bracket that.
    profiles = _profiles()
    profiles[0] = ServoProfile(0, 0, 1, -60.0, 60.0, "")
    profiles[1] = ServoProfile(1, 0, 1, -30.0, 85.0, "")
    profiles[2] = ServoProfile(2, 0, 1, 20.0, 140.0, "")
    result = gp.preflight(0, profiles)
    assert result.fits is True
    assert result.blocking == ()
    assert result.summary_lines() == ()


def test_preflight_checks_the_selected_legs_own_servos():
    # Leg 3's servos are 9/10/11 -- marking leg 0's must not satisfy it.
    profiles = _profiles()
    profiles[0] = ServoProfile(0, 0, 1, -60.0, 60.0, "")
    profiles[1] = ServoProfile(1, 0, 1, -30.0, 85.0, "")
    profiles[2] = ServoProfile(2, 0, 1, 20.0, 140.0, "")
    assert gp.servo_indices_for_leg(3) == (9, 10, 11)
    assert gp.preflight(0, profiles).fits is True
    assert gp.preflight(3, profiles).fits is False


def test_one_unmarked_direction_still_refuses():
    profiles = _profiles()
    profiles[0] = ServoProfile(0, 0, 1, -60.0, None, "")
    profiles[1] = ServoProfile(1, 0, 1, -30.0, 85.0, "")
    profiles[2] = ServoProfile(2, 0, 1, 20.0, 140.0, "")
    result = gp.preflight(0, profiles)
    assert result.fits is False
    assert "no MAX limit marked" in result.summary_lines()[0]


# --- slow motion default --------------------------------------------------


def test_default_slow_motion_is_actually_slow():
    # The first time this math drives hardware it must be slow enough to
    # hit RELEASE mid-motion. Pin the default rather than trusting that
    # nobody edits it upward casually.
    assert gp.DEFAULT_SLOW_MOTION <= 0.25
    cycle_s = 1.0 / (1.5 * gp.DEFAULT_SLOW_MOTION)  # BASE_PHASE_RATE_HZ at full speed
    assert cycle_s >= 4.0
    assert math.isclose(gp.MAX_SLOW_MOTION, 1.0)
