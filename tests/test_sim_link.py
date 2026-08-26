import time

import pytest

from simulator.sim_link import SimRobotLink
from transport.protocol import (
    BenchModeCommand,
    BodyHeightCommand,
    CalibrationModeCommand,
    PingCommand,
    StopCommand,
    TurnCommand,
    WalkCommand,
)

_SETTLE_S = 0.15  # a few ticks at the default 50Hz background rate


def test_works_with_zero_configuration():
    link = SimRobotLink()
    assert link.is_connected is True
    assert link.latest_telemetry() is not None
    link.close()


def test_send_returns_incrementing_sequence():
    link = SimRobotLink()
    seq_a = link.send(PingCommand())
    seq_b = link.send(PingCommand())
    assert seq_b == seq_a + 1
    link.close()


def test_send_after_close_raises():
    link = SimRobotLink()
    link.close()
    with pytest.raises(RuntimeError):
        link.send(PingCommand())


def test_walk_command_advances_gait_phase_over_time():
    link = SimRobotLink()
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=80))
    time.sleep(_SETTLE_S)
    snap = link.snapshot()
    assert snap.gait_phase > 0.0
    link.close()


def test_gait_phase_telemetry_null_while_idle():
    link = SimRobotLink()
    link.send(StopCommand())
    time.sleep(_SETTLE_S)
    assert link.latest_telemetry().gait_phase is None
    link.close()


def test_gait_phase_telemetry_populated_while_walking():
    link = SimRobotLink()
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=80))
    time.sleep(_SETTLE_S)
    assert link.latest_telemetry().gait_phase is not None
    link.close()


def test_body_moves_forward_while_walking():
    link = SimRobotLink()
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=100))
    time.sleep(_SETTLE_S)
    snap = link.snapshot()
    assert snap.body_x_mm > 0.0
    link.close()


def test_stop_after_walk_halts_further_body_movement():
    link = SimRobotLink()
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=100))
    time.sleep(_SETTLE_S)
    link.send(StopCommand())
    time.sleep(_SETTLE_S)
    x_after_stop = link.snapshot().body_x_mm
    time.sleep(_SETTLE_S)
    assert link.snapshot().body_x_mm == pytest.approx(x_after_stop, abs=1e-6)
    link.close()


def test_body_height_persists_across_resent_walk_commands():
    """Pins the exact bug caught and fixed in firmware/src/main.cpp:
    body_height must not reset just because a walk/turn command (which
    carries no height field of its own) is sent afterward."""
    link = SimRobotLink()
    link.send(BodyHeightCommand(height=80.0))
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=50))
    with link._gait_lock:
        assert link._current_body_height == 80.0


def test_link_timeout_freezes_gait():
    link = SimRobotLink(connection_timeout_s=0.1)
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=80))
    time.sleep(_SETTLE_S)
    phase_before = link.snapshot().gait_phase
    time.sleep(0.3)  # past connection_timeout_s, no further sends
    phase_after = link.snapshot().gait_phase
    assert phase_after == pytest.approx(phase_before, abs=1e-9)
    link.close()


def test_gait_resumes_after_fresh_command_post_timeout():
    """A real GUI keeps gait alive by continuously resending the current
    intent (the heartbeat) -- one fresh send only buys a resumed window
    up to connection_timeout_s again, so this checks with a settle time
    well inside that window, not past it (see
    test_link_timeout_freezes_gait for what happens with nothing
    resending)."""
    link = SimRobotLink(connection_timeout_s=0.5)
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=80))
    time.sleep(0.7)  # let the link time out
    phase_while_frozen = link.snapshot().gait_phase

    link.send(WalkCommand(vx=1.0, vy=0.0, speed=80))  # fresh command -- should resume
    time.sleep(_SETTLE_S)
    assert link.snapshot().gait_phase != pytest.approx(phase_while_frozen, abs=1e-9)
    link.close()


def test_bench_mode_freezes_gait():
    link = SimRobotLink()
    link.send(BenchModeCommand(armed=True))  # idle at this point -- allowed to arm
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=80))  # gait axes update, but gait itself won't tick while armed
    time.sleep(_SETTLE_S)
    assert link.snapshot().gait_phase == 0.0
    link.close()


def test_bench_mode_refuses_to_arm_while_walking():
    link = SimRobotLink()
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=80))
    telemetry = link.send_and_wait(BenchModeCommand(armed=True), timeout_s=1.0)
    assert telemetry is not None
    assert telemetry.ok is False
    assert telemetry.bench_armed is False
    link.close()


def test_calibration_mode_still_delegates_to_mock():
    link = SimRobotLink()
    telemetry = link.send_and_wait(CalibrationModeCommand(armed=True), timeout_s=1.0)
    assert telemetry is not None
    assert telemetry.calibration_armed is True
    link.close()


def test_turn_command_rotates_body():
    link = SimRobotLink()
    link.send(TurnCommand(rate=1.0, speed=100))
    time.sleep(_SETTLE_S)
    assert link.snapshot().heading_deg != 0.0
    link.close()
