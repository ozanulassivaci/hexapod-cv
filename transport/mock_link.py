"""MockRobotLink: no networking, no hardware. Records every command sent
and synthesizes a plausible telemetry reply synchronously, including a
simulation of the calibration arm/disarm gate (docs/protocol.md Section 6)
so GUI development against calibration workflows doesn't have to wait for
firmware either.

Meant to be lived with for weeks while parts ship -- inspectable state
(`sent_commands`, `last_command`), optional console logging, and a couple
of hooks (`set_fault`) for exercising states a real robot would only reach
through a fault condition.
"""

import dataclasses
import time

from transport.generated_constants import CALIBRATION_ARM_TIMEOUT_S, LINK_TIMEOUT_S, SEQUENCE_MODULUS
from transport.link import RobotLink
from transport.protocol import (
    BodyHeightCommand,
    CalibrateCommand,
    CalibrationModeCommand,
    Command,
    FaceCommand,
    PanTiltCommand,
    PingCommand,
    ReadOffsetsCommand,
    ServoOffset,
    StopCommand,
    Telemetry,
    TurnCommand,
    WalkCommand,
    WriteOffsetsCommand,
    next_sequence,
    SERVO_COUNT,
)

_MOTION_TYPES = (
    WalkCommand,
    TurnCommand,
    StopCommand,
    BodyHeightCommand,
    PanTiltCommand,
    FaceCommand,
)


class MockRobotLink(RobotLink):
    def __init__(
        self,
        console_log: bool = False,
        connection_timeout_s: float = LINK_TIMEOUT_S,
        calibration_arm_timeout_s: float = CALIBRATION_ARM_TIMEOUT_S,
    ) -> None:
        super().__init__(connection_timeout_s=connection_timeout_s)
        self.console_log = console_log
        self.sent_commands: list[Command] = []

        self._calibration_arm_timeout_s = calibration_arm_timeout_s
        self._seq = 0
        self._last_motion_command: Command = StopCommand()
        self._offsets: dict[int, ServoOffset] = {
            i: ServoOffset(i, 0, 1) for i in range(SERVO_COUNT)
        }
        self._calibration_armed_until: float | None = None
        self._closed = False

        # Seed telemetry so latest_telemetry()/is_connected work before the
        # first send() -- a mock "robot" is always on until told otherwise.
        # seq_echo is seeded one before the real counter's start (wrapping
        # to SEQUENCE_MODULUS - 1), not 0, so the first real send (seq=0)
        # is correctly treated as newer by the latest-wins check instead of
        # colliding with this seed value.
        self._record_telemetry(
            self._make_telemetry(seq_echo=SEQUENCE_MODULUS - 1, ok=True, error=None, offsets=None)
        )

    @property
    def last_command(self) -> Command | None:
        return self.sent_commands[-1] if self.sent_commands else None

    def send(self, command: Command) -> int:
        if self._closed:
            raise RuntimeError("send() called after close()")

        seq = self._seq
        self._seq = next_sequence(self._seq)
        self.sent_commands.append(command)
        if self.console_log:
            print(f"[MockRobotLink] seq={seq} {command!r}")

        ok, error, offsets_reply = self._apply(command)
        self._record_telemetry(
            self._make_telemetry(seq_echo=seq, ok=ok, error=error, offsets=offsets_reply)
        )
        return seq

    def set_fault(self, fault_flags: int) -> None:
        """Inject a fault flag into the next-reported telemetry, to test how
        calling code reacts to a fault state without touching real
        hardware. Takes effect immediately if telemetry already exists."""
        telemetry = self.latest_telemetry()
        if telemetry is None:
            return
        self._record_telemetry(
            dataclasses.replace(telemetry, fault_flags=fault_flags), allow_same_seq=True
        )

    def close(self) -> None:
        self._closed = True

    def _apply(self, command: Command):
        now = time.monotonic()
        armed = self._calibration_armed_until is not None and now < self._calibration_armed_until

        if isinstance(command, _MOTION_TYPES):
            self._last_motion_command = command
            return True, None, None

        if isinstance(command, PingCommand):
            return True, None, None

        if isinstance(command, CalibrationModeCommand):
            if command.armed:
                self._calibration_armed_until = now + self._calibration_arm_timeout_s
            else:
                self._calibration_armed_until = None
            return True, None, None

        if isinstance(command, CalibrateCommand):
            if not armed:
                return False, "calibration not armed", None
            self._offsets[command.servo_index] = ServoOffset(
                command.servo_index, command.offset_us, command.sign
            )
            self._calibration_armed_until = now + self._calibration_arm_timeout_s
            return True, None, None

        if isinstance(command, WriteOffsetsCommand):
            if not armed:
                return False, "calibration not armed", None
            for offset in command.offsets:
                self._offsets[offset.servo_index] = offset
            self._calibration_armed_until = now + self._calibration_arm_timeout_s
            return True, None, None

        if isinstance(command, ReadOffsetsCommand):
            return True, None, tuple(self._offsets[i] for i in range(SERVO_COUNT))

        return False, f"unhandled command type {type(command).__name__}", None

    def _make_telemetry(self, seq_echo: int, ok: bool, error: str | None, offsets) -> Telemetry:
        now = time.monotonic()
        armed = self._calibration_armed_until is not None and now < self._calibration_armed_until
        return Telemetry(
            seq_echo=seq_echo,
            ok=ok,
            error=error,
            fault_flags=0,
            gait_phase=None,
            rail_mv=None,
            link_timeout_s=LINK_TIMEOUT_S,
            calibration_armed=armed,
            last_applied=self._last_motion_command,
            offsets=offsets,
        )
