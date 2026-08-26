"""MainWindow: wires the video feed, robot link, and tracker together.

Threading model (see the design discussion this was built from): MJPEGStream
and RobotLink both already expose thread-safe "latest value" polling reads
from their own background threads, so this window needs no QThread, no
cross-thread signal emission for the core dataflow -- one QTimer tick,
firing on the Qt main thread by construction, polls both and updates the
widgets directly. Manual key intent is sent once per press/release
transition (not per OS auto-repeat); the transport's own heartbeat thread
is solely responsible for keeping that intent flowing at a steady rate.
"""

import math
import time
from enum import Enum, auto

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QHBoxLayout, QMainWindow, QTabWidget, QVBoxLayout, QWidget

from control.tracker import Tracker
from operator_config import OperatorConfig
from perception.detection import Detector
from simulator.sim_link import SimRobotLink
from simulator.sim_view import SimView
from stream.mjpeg_stream import MJPEGStream
from transport.link import RobotLink
from transport.protocol import (
    BodyHeightCommand,
    PanTiltCommand,
    PingCommand,
    StopCommand,
    TurnCommand,
    WalkCommand,
)
from ui.bench_tab import BenchTab
from ui.calibration_tab import CalibrationTab
from ui.control_panel import ControlPanel
from ui.log_panel import LogPanel
from ui.video_panel import VideoPanel

_PING_INTERVAL_S = 1.0
_PING_TIMEOUT_S = 2.0

# key -> (vx_delta, vy_delta). Diagonal presses sum both axes and are
# normalized to unit magnitude below, so diagonal walking isn't faster
# than cardinal walking.
_MOVE_KEYS = {
    Qt.Key_W: (1.0, 0.0), Qt.Key_Up: (1.0, 0.0),
    Qt.Key_S: (-1.0, 0.0), Qt.Key_Down: (-1.0, 0.0),
    Qt.Key_D: (0.0, 1.0), Qt.Key_Right: (0.0, 1.0),
    Qt.Key_A: (0.0, -1.0), Qt.Key_Left: (0.0, -1.0),
}
_TURN_KEYS = {
    Qt.Key_Q: -1.0,
    Qt.Key_E: 1.0,
}


class Mode(Enum):
    MANUAL = auto()
    AUTO_TRACK = auto()


class MainWindow(QMainWindow):
    def __init__(
        self,
        config: OperatorConfig,
        stream: MJPEGStream,
        detector: Detector,
        link: RobotLink,
        tracker: Tracker,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.config = config
        self.stream = stream
        self.detector = detector
        self.link = link
        self.tracker = tracker

        self._mode = Mode.MANUAL
        self._active_move_keys: set[int] = set()
        self._active_turn_keys: set[int] = set()
        self._auto_track_has_target = False

        self._fps = 0.0
        self._last_tick_time = time.monotonic()
        self._pending_ping: tuple[int, float] | None = None
        self._last_ping_sent_at = 0.0
        self._last_rtt_ms: float | None = None

        self.setWindowTitle("hexapod-cv operator")
        self.setFocusPolicy(Qt.StrongFocus)

        self.video_panel = VideoPanel(config.ui.window_width, config.ui.window_height)
        self.control_panel = ControlPanel()
        self.calibration_tab = CalibrationTab(link)
        self.bench_tab = BenchTab(
            link,
            dwell_timeout_s=config.bench.dwell_timeout_s,
            nudge_step_us=config.bench.nudge_step_us,
            default_sweep_min_us=config.bench.default_sweep_min_us,
            default_sweep_max_us=config.bench.default_sweep_max_us,
            default_sweep_duration_s=config.bench.default_sweep_duration_s,
        )
        self.log_panel = LogPanel()
        # Only meaningful against a simulated link -- shown as its own
        # tab (not always present) so mock/udp sessions aren't shown an
        # irrelevant view. None when link isn't a SimRobotLink.
        self.sim_view = SimView(link) if isinstance(link, SimRobotLink) else None

        self._build_layout()
        self._connect_signals()

        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self._on_tick)
        self._tick_timer.start(max(1, int(config.ui.tick_interval_s * 1000)))

        self._send_and_log(StopCommand(), source="startup")
        self.setFocus()

    # --- layout -------------------------------------------------------

    def _build_layout(self) -> None:
        operate_tab = QWidget()
        operate_layout = QHBoxLayout(operate_tab)
        operate_layout.addWidget(self.video_panel, 2)
        operate_layout.addWidget(self.control_panel, 1)

        tabs = QTabWidget()
        tabs.setFocusPolicy(Qt.NoFocus)
        tabs.addTab(operate_tab, "Operate")
        tabs.addTab(self.calibration_tab, "Calibrate")
        tabs.addTab(self.bench_tab, "Bench Test")
        if self.sim_view is not None:
            tabs.addTab(self.sim_view, "Simulator")

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addWidget(tabs, 1)
        layout.addWidget(self.log_panel)
        self.setCentralWidget(central)

    def _connect_signals(self) -> None:
        self.control_panel.body_height_changed.connect(self._on_body_height_changed)
        self.control_panel.pan_changed.connect(self._on_pan_tilt_changed)
        self.control_panel.tilt_changed.connect(self._on_pan_tilt_changed)
        self.control_panel.speed_changed.connect(self._on_speed_changed)
        self.control_panel.mode_toggle_requested.connect(self._on_mode_toggle_requested)
        self.control_panel.estop_clicked.connect(self._on_estop)
        self.calibration_tab.log_message.connect(self.log_panel.log)
        self.bench_tab.log_message.connect(self.log_panel.log)

    # --- keyboard: current intent lives here, not in a resend timer -------

    def keyPressEvent(self, event) -> None:
        if event.isAutoRepeat():
            return  # OS key-repeat must not produce a packet per event
        key = event.key()

        if key == Qt.Key_Space:
            self._on_estop()
            return

        if key in _MOVE_KEYS:
            self._ensure_manual_mode()
            self._active_move_keys.add(key)
            self._recompute_and_send_intent()
            return

        if key in _TURN_KEYS:
            self._ensure_manual_mode()
            self._active_turn_keys.add(key)
            self._recompute_and_send_intent()
            return

        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if event.isAutoRepeat():
            return
        key = event.key()

        if key in _MOVE_KEYS:
            self._active_move_keys.discard(key)
            self._recompute_and_send_intent()
            return

        if key in _TURN_KEYS:
            self._active_turn_keys.discard(key)
            self._recompute_and_send_intent()
            return

        super().keyReleaseEvent(event)

    def _recompute_and_send_intent(self) -> None:
        """Called once per key-state transition, never on a timer. Movement
        keys take priority over turn keys when both are held -- translation
        is the primary locomotion intent, turning in place is auxiliary."""
        if self._active_move_keys:
            vx = sum(_MOVE_KEYS[k][0] for k in self._active_move_keys)
            vy = sum(_MOVE_KEYS[k][1] for k in self._active_move_keys)
            magnitude = math.hypot(vx, vy)
            if magnitude > 1.0:
                vx, vy = vx / magnitude, vy / magnitude
            command = WalkCommand(vx=vx, vy=vy, speed=self.control_panel.current_speed())
        elif self._active_turn_keys:
            rate = max(-1.0, min(1.0, sum(_TURN_KEYS[k] for k in self._active_turn_keys)))
            command = TurnCommand(rate=rate, speed=self.control_panel.current_speed())
        else:
            command = StopCommand()
        self._send_and_log(command, source="manual")

    def _ensure_manual_mode(self) -> None:
        if self._mode == Mode.AUTO_TRACK:
            self._set_mode(Mode.MANUAL, reason="key input")

    def _on_estop(self) -> None:
        self._active_move_keys.clear()
        self._active_turn_keys.clear()
        self._ensure_manual_mode()
        self._send_and_log(StopCommand(), source="ESTOP")
        self.bench_tab.emergency_stop()

    # --- mode ------------------------------------------------------

    def _on_mode_toggle_requested(self, want_auto_track: bool) -> None:
        self._set_mode(Mode.AUTO_TRACK if want_auto_track else Mode.MANUAL, reason="operator")

    def _set_mode(self, mode: Mode, reason: str) -> None:
        if mode == self._mode:
            return
        self._mode = mode
        self.control_panel.set_mode(mode == Mode.AUTO_TRACK)
        if mode == Mode.MANUAL:
            self._active_move_keys.clear()
            self._active_turn_keys.clear()
            self.log_panel.log(f"mode -> MANUAL ({reason})")
            self._send_and_log(StopCommand(), source="mode")
        else:
            self._auto_track_has_target = False
            self.log_panel.log(f"mode -> AUTO_TRACK ({reason})")

    # --- sliders: body height / pan / tilt send live; speed only refreshes
    # whatever walk/turn intent is already active ------------------------

    def _on_body_height_changed(self, height: float) -> None:
        self.link.send(BodyHeightCommand(height=height))

    def _on_pan_tilt_changed(self, _value: float) -> None:
        command = PanTiltCommand(
            pan=self.control_panel.current_pan(), tilt=self.control_panel.current_tilt()
        )
        self.link.send(command)

    def _on_speed_changed(self, _value: float) -> None:
        if self._active_move_keys or self._active_turn_keys:
            self._recompute_and_send_intent()

    # --- central tick: display refresh + AUTO_TRACK drive ----------------

    def _on_tick(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick_time
        self._last_tick_time = now
        if dt > 0:
            self._fps = self._fps * 0.9 + (1.0 / dt) * 0.1

        frame, timestamp = self.stream.read()
        detections = []
        frame_age_ms = None
        if frame is not None:
            detections = self.detector.detect(frame)
            if timestamp is not None:
                frame_age_ms = (now - timestamp) * 1000
        self.video_panel.update_frame(frame, detections, self._fps, frame_age_ms)
        self.control_panel.set_camera_connected(self.stream.is_connected)

        if self._mode == Mode.AUTO_TRACK:
            self._drive_auto_track(detections)

        telemetry = self.link.latest_telemetry()
        self._refresh_rtt(telemetry)
        self.control_panel.set_link_connected(self.link.is_connected)
        self.control_panel.set_constants_warning(self.link.constants_warning)
        self.control_panel.set_telemetry(telemetry, self._last_rtt_ms)
        self.calibration_tab.tick(telemetry)
        self.bench_tab.tick(telemetry)
        if self.sim_view is not None:
            self.sim_view.tick()

    def _drive_auto_track(self, detections) -> None:
        has_target = bool(detections)
        if has_target != self._auto_track_has_target:
            self.log_panel.log("AUTO_TRACK: target acquired" if has_target else "AUTO_TRACK: target lost")
            self._auto_track_has_target = has_target

        command = self.tracker.decide(detections)
        self.link.send(command)
        self.control_panel.set_current_intent(command)
        # Not logged per-tick (would be dozens of lines/sec) -- the current
        # intent label already shows it live; only acquire/lose transitions
        # above are worth a permanent log line.

    def _refresh_rtt(self, telemetry) -> None:
        now = time.monotonic()
        if self._pending_ping is not None:
            seq, sent_at = self._pending_ping
            if telemetry is not None and telemetry.seq_echo == seq:
                self._last_rtt_ms = (now - sent_at) * 1000
                self._pending_ping = None
            elif now - sent_at > _PING_TIMEOUT_S:
                self._pending_ping = None

        # Only probe while genuinely idle: sending anything via link.send()
        # replaces the transport's resent "current command". Pinging while
        # a manual or AUTO_TRACK intent is active would clobber it and stop
        # the robot -- ping is for idle liveness/RTT, never for driving.
        idle = (
            self._mode == Mode.MANUAL
            and not self._active_move_keys
            and not self._active_turn_keys
        )
        if not idle:
            return

        if self._pending_ping is None and now - self._last_ping_sent_at >= _PING_INTERVAL_S:
            seq = self.link.send(PingCommand())
            self._pending_ping = (seq, now)
            self._last_ping_sent_at = now

    # --- shared send+log path for discrete, meaningful actions -----------

    def _send_and_log(self, command, source: str) -> None:
        self.link.send(command)
        self.control_panel.set_current_intent(command)
        self.log_panel.log(f"[{source}] {command!r}")

    def closeEvent(self, event) -> None:
        self.bench_tab.emergency_stop()  # don't leave a servo held away from neutral unattended
        self._tick_timer.stop()
        self.stream.stop()
        self.link.close()
        super().closeEvent(event)
