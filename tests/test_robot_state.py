import pytest

from robot.gait import DEFAULT_BODY_HEIGHT
from simulator.robot_state import RobotState


def test_initial_snapshot_three_stance_three_swing():
    state = RobotState()
    snap = state.snapshot(connected=True)
    assert len(snap.foot_positions) == 6
    assert len(snap.stance) == 6
    assert snap.stance.count(True) == 3
    assert snap.stance.count(False) == 3


def test_initial_snapshot_at_home_no_motion():
    state = RobotState(body_height=DEFAULT_BODY_HEIGHT)
    snap = state.snapshot(connected=True)
    assert snap.body_x_mm == 0.0
    assert snap.body_y_mm == 0.0
    assert snap.heading_deg == 0.0
    assert snap.gait_phase == 0.0


def test_step_advances_gait_phase_when_walking():
    state = RobotState()
    for _ in range(10):
        state.step(0.02, vx=1.0, vy=0.0, speed=50.0, rotation=0.0, body_height=50.0)
    assert state.snapshot(connected=True).gait_phase > 0.0


def test_step_does_not_advance_phase_when_idle():
    state = RobotState()
    for _ in range(10):
        state.step(0.02, vx=0.0, vy=0.0, speed=0.0, rotation=0.0, body_height=50.0)
    assert state.snapshot(connected=True).gait_phase == 0.0


def test_body_pose_dead_reckons_forward_when_walking():
    state = RobotState()
    for _ in range(50):
        state.step(0.02, vx=1.0, vy=0.0, speed=100.0, rotation=0.0, body_height=50.0)
    snap = state.snapshot(connected=True)
    assert snap.body_x_mm > 0.0
    assert snap.body_y_mm == pytest.approx(0.0, abs=1e-6)


def test_body_pose_unchanged_when_idle():
    state = RobotState()
    for _ in range(50):
        state.step(0.02, vx=0.0, vy=0.0, speed=0.0, rotation=0.0, body_height=50.0)
    snap = state.snapshot(connected=True)
    assert snap.body_x_mm == 0.0
    assert snap.body_y_mm == 0.0
    assert snap.heading_deg == 0.0


def test_body_heading_turns_when_rotating():
    state = RobotState()
    for _ in range(50):
        state.step(0.02, vx=0.0, vy=0.0, speed=100.0, rotation=1.0, body_height=50.0)
    snap = state.snapshot(connected=True)
    assert snap.heading_deg != 0.0
    assert snap.body_x_mm == pytest.approx(0.0, abs=1e-6)
    assert snap.body_y_mm == pytest.approx(0.0, abs=1e-6)


def test_snapshot_connected_flag_passthrough():
    state = RobotState()
    assert state.snapshot(connected=True).connected is True
    assert state.snapshot(connected=False).connected is False
