"""SimRobotLink: a RobotLink that actually runs gait over time, on its own
background thread at a fixed rate -- the "own thread, fixed rate, free-
runs off the latest command" shape MJPEGStream and UDPRobotLink already
establish, applied to a simulated robot instead of a real one.

Delegates calibration_mode/bench_mode/profile bookkeeping to an internal
MockRobotLink instance (composition, not inheritance) rather than
reimplementing that logic a third time -- MockRobotLink already has it
right (including the edge-triggered arming fix). What this class adds on
top is what MockRobotLink structurally cannot do: gait state that
actually advances over time, and a link-timeout that actually freezes it
-- MockRobotLink has no concept of elapsed-since-last-command at all,
which is exactly what's needed to make "stop sending and watch the
failsafe" observable.

Gait's inputs (current vx/vy/speed/rotation/body_height) are tracked
independently per axis, updated only by the command type that actually
carries it -- deliberately not read from MockRobotLink's last_applied
(a single most-recent-command echo), which would reset e.g. body height
to whatever a never-sent field defaults to on the very next resent walk
packet. This mirrors a real bug caught and fixed in
firmware/src/main.cpp; see that fix's commit message for the full story.
"""

import dataclasses
import threading
import time

from transport.generated_constants import LINK_TIMEOUT_S
from transport.link import RobotLink
from transport.mock_link import MockRobotLink
from transport.protocol import (
    BodyHeightCommand,
    Command,
    StopCommand,
    Telemetry,
    TurnCommand,
    WalkCommand,
)
from robot.gait import DEFAULT_BODY_HEIGHT, is_motion_idle
from simulator.robot_state import RobotSnapshot, RobotState

_DEFAULT_TICK_HZ = 50.0  # matches firmware's CONTROL_LOOP_INTERVAL_MS


class SimRobotLink(RobotLink):
    def __init__(self, connection_timeout_s: float = LINK_TIMEOUT_S, tick_hz: float = _DEFAULT_TICK_HZ) -> None:
        super().__init__(connection_timeout_s=connection_timeout_s)
        self._mock = MockRobotLink()
        self.state = RobotState()

        self._gait_lock = threading.Lock()
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_speed = 0.0
        self._current_rotation = 0.0
        self._current_body_height = DEFAULT_BODY_HEIGHT
        self._last_command_at: float | None = None  # None = never sent, mirrors LinkWatchdog::hasEverReceivedPacket

        self._tick_interval_s = 1.0 / tick_hz
        self._closed = False
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="SimRobotLink", daemon=True)
        self._thread.start()

        self._record_telemetry(self._augment(self._mock.latest_telemetry()))

    def send(self, command: Command) -> int:
        if self._closed:
            raise RuntimeError("send() called after close()")

        with self._gait_lock:
            self._last_command_at = time.monotonic()
            if isinstance(command, WalkCommand):
                self._current_vx, self._current_vy, self._current_speed = command.vx, command.vy, command.speed
                self._current_rotation = 0.0
            elif isinstance(command, TurnCommand):
                self._current_vx, self._current_vy = 0.0, 0.0
                self._current_speed, self._current_rotation = command.speed, command.rate
            elif isinstance(command, StopCommand):
                self._current_vx = self._current_vy = self._current_speed = self._current_rotation = 0.0
            elif isinstance(command, BodyHeightCommand):
                self._current_body_height = command.height
            # PanTilt/Face/calibration/bench commands don't touch any gait axis.

        seq = self._mock.send(command)
        self._record_telemetry(self._augment(self._mock.latest_telemetry()))
        return seq

    def close(self) -> None:
        self._closed = True
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._mock.close()

    def snapshot(self) -> RobotSnapshot:
        """What sim_view.py polls -- a full, safe-to-read-from-any-thread
        picture of body pose, gait phase, and per-leg stance/swing."""
        return self.state.snapshot(connected=self.is_connected)

    # --- background stepping ---------------------------------------------

    def _run(self) -> None:
        last_tick = time.monotonic()
        while not self._stop_event.wait(self._tick_interval_s):
            now = time.monotonic()
            dt = now - last_tick
            last_tick = now

            with self._gait_lock:
                last_command_at = self._last_command_at
                vx, vy, speed, rotation, height = (
                    self._current_vx,
                    self._current_vy,
                    self._current_speed,
                    self._current_rotation,
                    self._current_body_height,
                )

            timed_out = last_command_at is None or (now - last_command_at) > self._connection_timeout_s
            latest = self._mock.latest_telemetry()
            bench_armed = latest is not None and latest.bench_armed
            if timed_out or bench_armed:
                continue  # frozen -- mirrors firmware's loop() gaitShouldRun gate

            self.state.step(dt, vx, vy, speed, rotation, height)

    def _augment(self, mock_telemetry: Telemetry) -> Telemetry:
        """MockRobotLink's telemetry, with gait_phase populated for real
        (mock always reports it as null -- see docs/protocol.md Section
        6's audit note) using this link's own current motion axes, same
        null-while-idle rule firmware's buildBaseTelemetry uses."""
        with self._gait_lock:
            vx, vy, speed, rotation = (
                self._current_vx,
                self._current_vy,
                self._current_speed,
                self._current_rotation,
            )
        idle = is_motion_idle(vx, vy, speed, rotation)
        gait_phase = None if idle else self.state.snapshot(connected=True).gait_phase
        return dataclasses.replace(mock_telemetry, gait_phase=gait_phase)
