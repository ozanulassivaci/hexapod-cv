import time

import pytest

from transport.mock_link import MockRobotLink
from transport.protocol import (
    CalibrateCommand,
    CalibrationModeCommand,
    FAULT_LINK_TIMEOUT,
    PingCommand,
    ReadOffsetsCommand,
    ServoOffset,
    StopCommand,
    WalkCommand,
    WriteOffsetsCommand,
)


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


def test_read_offsets_defaults_to_zero_and_sign_one():
    link = MockRobotLink()
    link.send(ReadOffsetsCommand())
    offsets = link.latest_telemetry().offsets
    assert len(offsets) == 18
    assert all(o.offset_us == 0 and o.sign == 1 for o in offsets)


def test_read_offsets_always_allowed_even_unarmed():
    link = MockRobotLink()
    link.send(ReadOffsetsCommand())
    assert link.latest_telemetry().ok is True


def test_write_offsets_round_trips_through_read():
    link = MockRobotLink()
    link.send(CalibrationModeCommand(armed=True))
    link.send(
        WriteOffsetsCommand(
            offsets=[ServoOffset(0, -50, -1), ServoOffset(4, 30, 1)]
        )
    )
    link.send(ReadOffsetsCommand())
    offsets = link.latest_telemetry().offsets
    assert offsets[0] == ServoOffset(0, -50, -1)
    assert offsets[4] == ServoOffset(4, 30, 1)
    assert offsets[1] == ServoOffset(1, 0, 1)  # untouched entries keep defaults


def test_write_offsets_rejected_when_not_armed():
    link = MockRobotLink()
    link.send(WriteOffsetsCommand(offsets=[ServoOffset(0, 1, 1)]))
    telemetry = link.latest_telemetry()
    assert telemetry.ok is False
    assert telemetry.error == "calibration not armed"


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
