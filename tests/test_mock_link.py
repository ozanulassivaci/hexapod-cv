import time

import pytest

from transport.mock_link import MockRobotLink
from transport.protocol import (
    BenchHealthNoteCommand,
    BenchModeCommand,
    BenchPulseCommand,
    BenchReleaseCommand,
    BodyHeightCommand,
    CalibrateCommand,
    CalibrationModeCommand,
    FAULT_LINK_TIMEOUT,
    LimitBound,
    PingCommand,
    ReadOffsetsCommand,
    RecordLimitCommand,
    ServoProfile,
    StopCommand,
    TurnCommand,
    WalkCommand,
    WriteOffsetsCommand,
)


def test_robot_assembled_defaults_to_false():
    link = MockRobotLink()
    assert link.latest_telemetry().robot_assembled is False


def test_robot_assembled_reports_constructor_value():
    link = MockRobotLink(robot_assembled=True)
    assert link.latest_telemetry().robot_assembled is True
    link.send(PingCommand())
    assert link.latest_telemetry().robot_assembled is True  # fixed, not command-changeable


def test_works_with_zero_configuration():
    link = MockRobotLink()
    assert link.is_connected is True
    assert link.latest_telemetry() is not None
    link.close()


def test_records_sent_commands():
    link = MockRobotLink()
    cmd1 = WalkCommand(vx=0.5, vy=0, speed=10)
    cmd2 = StopCommand()
    link.send(cmd1)
    link.send(cmd2)
    assert link.sent_commands == [cmd1, cmd2]
    assert link.last_command == cmd2


def test_last_command_none_before_any_send():
    link = MockRobotLink()
    assert link.last_command is None


def test_console_log_prints(capsys):
    link = MockRobotLink(console_log=True)
    link.send(WalkCommand(vx=0.1, vy=0, speed=5))
    captured = capsys.readouterr()
    assert "MockRobotLink" in captured.out
    assert "WalkCommand" in captured.out


def test_console_log_off_by_default_is_silent(capsys):
    link = MockRobotLink()
    link.send(WalkCommand(vx=0.1, vy=0, speed=5))
    captured = capsys.readouterr()
    assert captured.out == ""


def test_last_applied_reflects_motion_commands():
    link = MockRobotLink()
    link.send(WalkCommand(vx=0.4, vy=-0.1, speed=20))
    assert link.latest_telemetry().last_applied == WalkCommand(vx=0.4, vy=-0.1, speed=20)
    link.send(StopCommand())
    assert link.latest_telemetry().last_applied == StopCommand()


def test_bench_pulse_and_record_limit_do_not_touch_last_applied():
    link = MockRobotLink()
    link.send(WalkCommand(vx=0.4, vy=-0.1, speed=20))
    link.send(BenchModeCommand(armed=True))
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500))
    link.send(RecordLimitCommand(servo_index=0, bound=LimitBound.MIN, pulse_us=1000))
    assert link.latest_telemetry().last_applied == WalkCommand(vx=0.4, vy=-0.1, speed=20)


def test_calibrate_rejected_when_not_armed():
    link = MockRobotLink()
    link.send(CalibrateCommand(servo_index=2, offset_us=10, sign=1))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "calibration not armed"


def test_calibrate_accepted_when_armed():
    link = MockRobotLink()
    link.send(CalibrationModeCommand(armed=True))
    assert link.latest_telemetry().calibration_armed is True

    link.send(CalibrateCommand(servo_index=2, offset_us=10, sign=1))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is True
    assert telemetry.error is None


def test_calibration_mode_disarm():
    link = MockRobotLink()
    link.send(CalibrationModeCommand(armed=True))
    link.send(CalibrationModeCommand(armed=False))
    assert link.latest_telemetry().calibration_armed is False

    link.send(CalibrateCommand(servo_index=0, offset_us=1, sign=1))
    assert link.latest_telemetry().ok is False


def test_calibration_auto_disarms_after_timeout():
    link = MockRobotLink(calibration_arm_timeout_s=0.05)
    link.send(CalibrationModeCommand(armed=True))
    assert link.latest_telemetry().calibration_armed is True

    time.sleep(0.1)

    # Reading the armed state itself doesn't refresh it; the timeout has
    # already elapsed by the time this ping is processed.
    link.send(PingCommand())
    assert link.latest_telemetry().calibration_armed is False

    link.send(CalibrateCommand(servo_index=0, offset_us=1, sign=1))
    assert link.latest_telemetry().ok is False


def test_calibration_mode_resend_does_not_extend_arm_window():
    """Edge-triggered, matching firmware's ArmGate::arm(): a resent
    calibration_mode(armed=True) (indistinguishable on the wire from a
    deliberate re-click, since the transport resends whatever was last
    sent) must not extend the window -- only a gated write (calibrate/
    write_offsets) does that. See docs/protocol.md Section 6."""
    link = MockRobotLink(calibration_arm_timeout_s=0.2)
    link.send(CalibrationModeCommand(armed=True))
    time.sleep(0.1)
    link.send(CalibrationModeCommand(armed=True))  # simulated resend, must not refresh
    time.sleep(0.15)
    # 0.25s since the original arm (> 0.2s timeout); if the resend had
    # refreshed the window, only 0.15s would have elapsed since then and
    # this would still be armed. It must not be.
    link.send(CalibrateCommand(servo_index=0, offset_us=1, sign=1))
    assert link.latest_telemetry().ok is False
    assert link.latest_telemetry().error == "calibration not armed"


def test_calibrate_while_armed_refreshes_arm_window():
    link = MockRobotLink(calibration_arm_timeout_s=0.2)
    link.send(CalibrationModeCommand(armed=True))
    time.sleep(0.1)
    link.send(CalibrateCommand(servo_index=0, offset_us=1, sign=1))  # refreshes window
    time.sleep(0.15)
    # Total elapsed since arm is 0.25s (> 0.2s timeout), but only 0.15s
    # since the refresh -- should still be armed.
    link.send(CalibrateCommand(servo_index=1, offset_us=2, sign=1))
    assert link.latest_telemetry().ok is True


def test_read_offsets_defaults_to_zero_sign_one_and_no_recorded_limits():
    link = MockRobotLink()
    link.send(ReadOffsetsCommand())
    profiles = link.latest_telemetry().profiles
    assert len(profiles) == 18
    assert all(p.offset_us == 0 and p.sign == 1 for p in profiles)
    assert all(p.min_deg_from_neutral is None and p.max_deg_from_neutral is None for p in profiles)


def test_read_offsets_always_allowed_even_unarmed():
    link = MockRobotLink()
    link.send(ReadOffsetsCommand())
    assert link.latest_telemetry().ok is True


def test_write_offsets_round_trips_through_read():
    link = MockRobotLink()
    link.send(CalibrationModeCommand(armed=True))
    link.send(
        WriteOffsetsCommand(
            offsets=[ServoProfile(0, -50, -1), ServoProfile(4, 30, 1)]
        )
    )
    link.send(ReadOffsetsCommand())
    profiles = link.latest_telemetry().profiles
    assert profiles[0].offset_us == -50 and profiles[0].sign == -1
    assert profiles[4].offset_us == 30 and profiles[4].sign == 1
    assert profiles[1].offset_us == 0 and profiles[1].sign == 1  # untouched entries keep defaults


def test_write_offsets_rejected_when_not_armed():
    link = MockRobotLink()
    link.send(WriteOffsetsCommand(offsets=[ServoProfile(0, 1, 1)]))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "calibration not armed"


# --- bench/gait mutual exclusion ----------------------------------------


def test_bench_mode_refuses_to_arm_while_walk_active():
    link = MockRobotLink()
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=50))
    link.send(BenchModeCommand(armed=True))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "gait active, cannot arm bench mode"
    assert telemetry.bench_armed is False


def test_bench_mode_refuses_to_arm_while_turn_active():
    link = MockRobotLink()
    link.send(TurnCommand(rate=0.5, speed=30))
    link.send(BenchModeCommand(armed=True))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.bench_armed is False


def test_bench_mode_arms_when_walk_speed_is_zero():
    link = MockRobotLink()
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=0))  # speed 0 -- idle regardless of vx/vy
    link.send(BenchModeCommand(armed=True))
    assert link.latest_telemetry().bench_armed is True


def test_bench_mode_arms_after_stop():
    link = MockRobotLink()
    link.send(WalkCommand(vx=1.0, vy=0.0, speed=50))
    link.send(StopCommand())
    link.send(BenchModeCommand(armed=True))
    assert link.latest_telemetry().bench_armed is True


def test_body_height_does_not_count_as_gait_active():
    """A body_height command alone (no walk/turn) touches no gait axis --
    must not itself block arming bench mode."""
    link = MockRobotLink()
    link.send(BodyHeightCommand(height=80.0))
    link.send(BenchModeCommand(armed=True))
    assert link.latest_telemetry().bench_armed is True


# --- bench mode --------------------------------------------------------


def test_bench_pulse_rejected_when_not_armed():
    link = MockRobotLink()
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "bench mode not armed"


def test_bench_pulse_accepted_when_armed():
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500))
    assert link.latest_telemetry().ok is True


def test_bench_mode_independent_of_calibration_mode():
    """Arming one gate must not arm or disarm the other."""
    link = MockRobotLink()
    link.send(CalibrationModeCommand(armed=True))
    assert link.latest_telemetry().calibration_armed is True
    assert link.latest_telemetry().bench_armed is False

    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500))
    assert link.latest_telemetry().ok is False  # calibration armed doesn't imply bench armed

    link.send(BenchModeCommand(armed=True))
    assert link.latest_telemetry().calibration_armed is True
    assert link.latest_telemetry().bench_armed is True


def test_bench_auto_disarms_after_timeout():
    link = MockRobotLink(bench_arm_timeout_s=0.05)
    link.send(BenchModeCommand(armed=True))
    assert link.latest_telemetry().bench_armed is True

    time.sleep(0.1)
    link.send(PingCommand())
    assert link.latest_telemetry().bench_armed is False

    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500))
    assert link.latest_telemetry().ok is False


def test_record_limit_rejected_when_bench_not_armed():
    link = MockRobotLink()
    link.send(RecordLimitCommand(servo_index=0, bound=LimitBound.MIN, pulse_us=1000))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "bench mode not armed"


def test_record_limit_round_trips_through_read():
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(RecordLimitCommand(servo_index=6, bound=LimitBound.MIN, pulse_us=950))
    link.send(RecordLimitCommand(servo_index=6, bound=LimitBound.MAX, pulse_us=2050))
    link.send(ReadOffsetsCommand())
    profile = link.latest_telemetry().profiles[6]
    # servo_index=6 is a coxa (6 % 3 == 0, neutral=NEUTRAL_PULSE_US=1500):
    # (950-1500)/(2000/180) = -49.5, (2050-1500)/(2000/180) = +49.5.
    assert profile.min_deg_from_neutral == pytest.approx(-49.5)
    assert profile.max_deg_from_neutral == pytest.approx(49.5)


def test_record_limit_rejects_min_at_or_past_existing_max():
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(RecordLimitCommand(servo_index=0, bound=LimitBound.MAX, pulse_us=2000))
    link.send(RecordLimitCommand(servo_index=0, bound=LimitBound.MIN, pulse_us=2000))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert "must be <" in telemetry.error


def test_calibrate_write_preserves_bench_recorded_limits_and_note():
    """A calibrate/write_offsets write must never wipe out limits or a
    health note already recorded for that servo by bench testing."""
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(RecordLimitCommand(servo_index=5, bound=LimitBound.MIN, pulse_us=950))
    link.send(RecordLimitCommand(servo_index=5, bound=LimitBound.MAX, pulse_us=2050))
    link.send(BenchHealthNoteCommand(servo_index=5, note="slight buzz near max"))

    link.send(CalibrationModeCommand(armed=True))
    link.send(CalibrateCommand(servo_index=5, offset_us=30, sign=-1))

    link.send(ReadOffsetsCommand())
    profile = link.latest_telemetry().profiles[5]
    assert profile.offset_us == 30
    assert profile.sign == -1
    # servo_index=5 is a tibia (5 % 3 == 2, neutral=TIBIA_NEUTRAL_PULSE_US=500):
    # (950-500)/(2000/180) = 40.5, (2050-500)/(2000/180) = 139.5.
    assert profile.min_deg_from_neutral == pytest.approx(40.5)
    assert profile.max_deg_from_neutral == pytest.approx(139.5)
    assert profile.note == "slight buzz near max"


def test_bench_health_note_not_gated_by_bench_armed():
    link = MockRobotLink()
    link.send(BenchHealthNoteCommand(servo_index=3, note="dead"))
    assert link.latest_telemetry().ok is True

    link.send(ReadOffsetsCommand())
    assert link.latest_telemetry().profiles[3].note == "dead"


def test_bench_mode_resend_does_not_extend_arm_window():
    """Same edge-triggered rule as calibration_mode, mirrored for bench
    mode's independent gate."""
    link = MockRobotLink(bench_arm_timeout_s=0.2)
    link.send(BenchModeCommand(armed=True))
    time.sleep(0.1)
    link.send(BenchModeCommand(armed=True))  # simulated resend, must not refresh
    time.sleep(0.15)
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500))
    assert link.latest_telemetry().ok is False
    assert link.latest_telemetry().error == "bench mode not armed"


def test_bench_pulse_rejected_below_recorded_min_for_servo():
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(RecordLimitCommand(servo_index=2, bound=LimitBound.MIN, pulse_us=1000))
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=900, servo_index=2))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "pulse below recorded min for this servo"


def test_bench_pulse_rejected_above_recorded_max_for_servo():
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(RecordLimitCommand(servo_index=2, bound=LimitBound.MAX, pulse_us=2000))
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=2100, servo_index=2))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "pulse above recorded max for this servo"


def test_bench_pulse_within_recorded_limits_accepted():
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(RecordLimitCommand(servo_index=2, bound=LimitBound.MIN, pulse_us=1000))
    link.send(RecordLimitCommand(servo_index=2, bound=LimitBound.MAX, pulse_us=2000))
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500, servo_index=2))
    assert link.latest_telemetry().ok is True


def test_bench_release_rejected_when_bench_not_armed():
    link = MockRobotLink()
    link.send(BenchReleaseCommand(board=0x40, channel=0))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "bench mode not armed"


def test_bench_release_accepted_when_armed():
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(BenchReleaseCommand(board=0x40, channel=3))
    assert link.latest_telemetry().ok is True


def test_bench_release_refreshes_the_arm_window():
    link = MockRobotLink(bench_arm_timeout_s=0.2)
    link.send(BenchModeCommand(armed=True))
    time.sleep(0.1)
    link.send(BenchReleaseCommand(board=0x40, channel=0))  # refreshes window
    time.sleep(0.15)
    # Total elapsed since arm is 0.25s (> 0.2s timeout), but only 0.15s
    # since the refresh -- should still be armed.
    link.send(BenchReleaseCommand(board=0x40, channel=0))
    assert link.latest_telemetry().ok is True


def test_bench_pulse_without_servo_index_skips_limit_enforcement():
    """No servo_index means no identity to look limits up under -- same
    as firmware's GatedServoDriver, only the generic protocol-level bound
    applies (already enforced at construction)."""
    link = MockRobotLink()
    link.send(BenchModeCommand(armed=True))
    link.send(RecordLimitCommand(servo_index=2, bound=LimitBound.MIN, pulse_us=1000))
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=600))
    assert link.latest_telemetry().ok is True


def test_bench_pulse_rejected_by_limit_does_not_refresh_arm_window():
    link = MockRobotLink(bench_arm_timeout_s=0.2)
    link.send(BenchModeCommand(armed=True))
    link.send(RecordLimitCommand(servo_index=2, bound=LimitBound.MIN, pulse_us=1000))
    time.sleep(0.15)
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=900, servo_index=2))  # rejected
    time.sleep(0.1)
    # 0.25s since arm, > 0.2s timeout -- the rejected pulse above must not
    # have refreshed the window, matching firmware (refresh happens after
    # GatedServoDriver::commandPulse succeeds, not before).
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "bench mode not armed"


def test_bench_pulse_refreshes_bench_arm_window():
    link = MockRobotLink(bench_arm_timeout_s=0.2)
    link.send(BenchModeCommand(armed=True))
    time.sleep(0.1)
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1500))  # refreshes window
    time.sleep(0.15)
    # Total elapsed since arm is 0.25s (> 0.2s timeout), but only 0.15s
    # since the refresh -- should still be armed.
    link.send(BenchPulseCommand(board=0x40, channel=0, pulse_us=1600))
    assert link.latest_telemetry().ok is True


def test_set_fault_reflected_in_telemetry():
    link = MockRobotLink()
    link.send(PingCommand())
    link.set_fault(FAULT_LINK_TIMEOUT)
    assert link.latest_telemetry().fault_flags == FAULT_LINK_TIMEOUT


def test_send_after_close_raises():
    link = MockRobotLink()
    link.close()
    with pytest.raises(RuntimeError):
        link.send(PingCommand())


def test_send_returns_incrementing_sequence():
    link = MockRobotLink()
    seq_a = link.send(PingCommand())
    seq_b = link.send(PingCommand())
    assert seq_b == seq_a + 1


def test_wait_for_ack_returns_matching_telemetry():
    link = MockRobotLink()
    seq = link.send(WalkCommand(vx=0.2, vy=0, speed=10))
    telemetry = link.wait_for_ack(seq, timeout_s=1.0)
    assert telemetry is not None
    assert telemetry.seq_echo == seq


def test_send_and_wait_convenience():
    link = MockRobotLink()
    telemetry = link.send_and_wait(CalibrationModeCommand(armed=True), timeout_s=1.0)
    assert telemetry is not None
    assert telemetry.ok is True
    assert telemetry.calibration_armed is True
