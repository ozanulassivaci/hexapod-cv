"""MockRobotLink: no networking, no hardware. Records every command sent
and synthesizes a plausible telemetry reply synchronously, including a
simulation of both arm/disarm gates -- calibration_mode (docs/protocol.md
Section 6) and bench_mode (Section 8) -- so GUI development against either
workflow doesn't have to wait for firmware.

Meant to be lived with for weeks while parts ship -- inspectable state
(`sent_commands`, `last_command`), optional console logging, and a couple
of hooks (`set_fault`) for exercising states a real robot would only reach
through a fault condition.
"""

import dataclasses
import time

from transport.generated_constants import (
    BENCH_ARM_TIMEOUT_S,
    CALIBRATION_ARM_TIMEOUT_S,
    LINK_TIMEOUT_S,
    SEQUENCE_MODULUS,
)
from transport.link import RobotLink
from transport.protocol import (
    BenchHealthNoteCommand,
    BenchModeCommand,
    BenchPulseCommand,
    BodyHeightCommand,
    CalibrateCommand,
    CalibrationModeCommand,
    Command,
    FaceCommand,
    LimitBound,
    PanTiltCommand,
    PingCommand,
    ProtocolError,
    ReadOffsetsCommand,
    RecordLimitCommand,
    SERVO_COUNT,
    ServoProfile,
    StopCommand,
    Telemetry,
    TurnCommand,
    WalkCommand,
    WriteOffsetsCommand,
    next_sequence,
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
        bench_arm_timeout_s: float = BENCH_ARM_TIMEOUT_S,
    ) -> None:
        super().__init__(connection_timeout_s=connection_timeout_s)
        self.console_log = console_log
        self.sent_commands: list[Command] = []

        self._calibration_arm_timeout_s = calibration_arm_timeout_s
        self._bench_arm_timeout_s = bench_arm_timeout_s
        self._seq = 0
        self._last_motion_command: Command = StopCommand()
        self._profiles: dict[int, ServoProfile] = {
            i: ServoProfile(i, 0, 1, None, None, "") for i in range(SERVO_COUNT)
        }
        self._calibration_armed_until: float | None = None
        self._bench_armed_until: float | None = None
        self._closed = False

        # Seed telemetry so latest_telemetry()/is_connected work before the
        # first send() -- a mock "robot" is always on until told otherwise.
        # seq_echo is seeded one before the real counter's start (wrapping
        # to SEQUENCE_MODULUS - 1), not 0, so the first real send (seq=0)
        # is correctly treated as newer by the latest-wins check instead of
        # colliding with this seed value.
        self._record_telemetry(
            self._make_telemetry(seq_echo=SEQUENCE_MODULUS - 1, ok=True, error=None, profiles=None)
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

        ok, error, profiles_reply = self._apply(command)
        self._record_telemetry(
            self._make_telemetry(seq_echo=seq, ok=ok, error=error, profiles=profiles_reply)
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
        calibration_armed = (
            self._calibration_armed_until is not None and now < self._calibration_armed_until
        )
        bench_armed = self._bench_armed_until is not None and now < self._bench_armed_until

        if isinstance(command, _MOTION_TYPES):
            self._last_motion_command = command
            return True, None, None

        if isinstance(command, PingCommand):
            return True, None, None

        if isinstance(command, CalibrationModeCommand):
            if command.armed:
                # Edge-triggered, matching firmware's ArmGate::arm() --
                # only the false->true transition opens a fresh window.
                # The transport resends whatever was last sent regardless
                # of type, so a resent armed=true is indistinguishable
                # from a deliberate re-arm; refreshing on every receipt
                # would mean an idle-but-armed session with a healthy
                # link never auto-disarms (docs/protocol.md Section 6).
                if not calibration_armed:
                    self._calibration_armed_until = now + self._calibration_arm_timeout_s
            else:
                self._calibration_armed_until = None
            return True, None, None

        if isinstance(command, CalibrateCommand):
            if not calibration_armed:
                return False, "calibration not armed", None
            # Merge onto the existing profile -- offset_us/sign only. A
            # calibrate/write_offsets write must never touch (or silently
            # wipe) min_pulse_us/max_pulse_us/note, which are bench mode's
            # concern and may already be recorded for this servo.
            existing = self._profiles[command.servo_index]
            self._profiles[command.servo_index] = dataclasses.replace(
                existing, offset_us=command.offset_us, sign=command.sign
            )
            self._calibration_armed_until = now + self._calibration_arm_timeout_s
            return True, None, None

        if isinstance(command, WriteOffsetsCommand):
            if not calibration_armed:
                return False, "calibration not armed", None
            for offset in command.offsets:
                existing = self._profiles[offset.servo_index]
                self._profiles[offset.servo_index] = dataclasses.replace(
                    existing, offset_us=offset.offset_us, sign=offset.sign
                )
            self._calibration_armed_until = now + self._calibration_arm_timeout_s
            return True, None, None

        if isinstance(command, ReadOffsetsCommand):
            return True, None, tuple(self._profiles[i] for i in range(SERVO_COUNT))

        if isinstance(command, BenchModeCommand):
            if command.armed:
                # Same edge-only reasoning as calibration_mode above.
                if not bench_armed:
                    self._bench_armed_until = now + self._bench_arm_timeout_s
            else:
                self._bench_armed_until = None
            return True, None, None

        if isinstance(command, BenchPulseCommand):
            if not bench_armed:
                return False, "bench mode not armed", None
            if command.servo_index is not None:
                profile = self._profiles[command.servo_index]
                if profile.min_pulse_us is not None and command.pulse_us < profile.min_pulse_us:
                    return False, "pulse below recorded min for this servo", None
                if profile.max_pulse_us is not None and command.pulse_us > profile.max_pulse_us:
                    return False, "pulse above recorded max for this servo", None
            self._bench_armed_until = now + self._bench_arm_timeout_s
            return True, None, None

        if isinstance(command, RecordLimitCommand):
            if not bench_armed:
                return False, "bench mode not armed", None
            existing = self._profiles[command.servo_index]
            try:
                if command.bound == LimitBound.MIN:
                    updated = dataclasses.replace(existing, min_pulse_us=command.pulse_us)
                else:
                    updated = dataclasses.replace(existing, max_pulse_us=command.pulse_us)
            except ProtocolError as exc:
                return False, str(exc), None
            self._profiles[command.servo_index] = updated
            self._bench_armed_until = now + self._bench_arm_timeout_s
            return True, None, None

        if isinstance(command, BenchHealthNoteCommand):
            # Not gated by bench_armed -- advisory record-keeping, not an
            # actuation or safety-relevant write. Doesn't refresh the bench
            # arm window either, for the same reason.
            existing = self._profiles[command.servo_index]
            self._profiles[command.servo_index] = dataclasses.replace(existing, note=command.note)
            return True, None, None

        return False, f"unhandled command type {type(command).__name__}", None

    def _make_telemetry(self, seq_echo: int, ok: bool, error: str | None, profiles) -> Telemetry:
        now = time.monotonic()
        calibration_armed = (
            self._calibration_armed_until is not None and now < self._calibration_armed_until
        )
        bench_armed = self._bench_armed_until is not None and now < self._bench_armed_until
        return Telemetry(
            seq_echo=seq_echo,
            ok=ok,
            error=error,
            fault_flags=0,
            gait_phase=None,
            rail_mv=None,
            link_timeout_s=LINK_TIMEOUT_S,
            calibration_armed=calibration_armed,
            bench_armed=bench_armed,
            last_applied=self._last_motion_command,
            profiles=profiles,
        )
