"""Right-hand control panel: connection status, telemetry readout, key
legend + current intent, sliders, mode toggle, emergency stop.

This widget only displays state and emits signals for user-initiated
actions (slider moves, mode toggle, e-stop click) -- it never calls
RobotLink itself. ui/main_window.py owns every actual send() call, so
there is exactly one place in the codebase that decides what gets
transmitted and when.
"""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from PySide6.QtCore import Qt

from transport.protocol import Command, FAULT_NONE, Telemetry, has_fault
from transport.protocol import FAULT_LINK_TIMEOUT, FAULT_ESTOP, FAULT_SERVO_FAULT, FAULT_BROWNOUT

_CONNECTED_STYLE = "background-color: #1b5e20; color: white; padding: 4px; font-weight: bold;"
_DISCONNECTED_STYLE = "background-color: #b71c1c; color: white; padding: 4px; font-weight: bold;"
_WARNING_STYLE = "background-color: #e65100; color: white; padding: 6px; font-weight: bold;"
# Neither safety mode is "wrong" on its own -- these are informational,
# not good/bad like CAMERA/LINK's green/red, so a distinct color family
# (not red/green) on purpose. See set_safety_mode()'s docstring for why
# this is never a warning even when it might be stale.
_SAFETY_BENCH_STYLE = "background-color: #01579b; color: white; padding: 6px; font-weight: bold;"
_SAFETY_ASSEMBLED_STYLE = "background-color: #4a148c; color: white; padding: 6px; font-weight: bold;"
_SAFETY_UNKNOWN_STYLE = "background-color: #424242; color: white; padding: 6px; font-weight: bold;"
_ESTOP_STYLE = (
    "background-color: #b71c1c; color: white; font-weight: bold; font-size: 20px; padding: 16px;"
)

_FAULT_NAMES = [
    (FAULT_LINK_TIMEOUT, "LINK_TIMEOUT"),
    (FAULT_ESTOP, "ESTOP"),
    (FAULT_SERVO_FAULT, "SERVO_FAULT"),
    (FAULT_BROWNOUT, "BROWNOUT"),
]


def _fault_flags_text(fault_flags: int) -> str:
    if fault_flags == FAULT_NONE:
        return "none"
    names = [name for bit, name in _FAULT_NAMES if has_fault(fault_flags, bit)]
    unknown = fault_flags & ~sum(bit for bit, _ in _FAULT_NAMES)
    if unknown:
        names.append(f"unknown(0x{unknown:x})")
    return ", ".join(names) if names else f"0x{fault_flags:x}"


class ControlPanel(QWidget):
    speed_changed = Signal(float)
    body_height_changed = Signal(float)
    pan_changed = Signal(float)
    tilt_changed = Signal(float)
    mode_toggle_requested = Signal(bool)  # True requests AUTO_TRACK
    estop_clicked = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)

        layout.addWidget(self._build_status_section())
        layout.addWidget(self._build_telemetry_section())
        layout.addWidget(self._build_keys_section())
        layout.addWidget(self._build_sliders_section())
        layout.addWidget(self._build_mode_section())
        layout.addWidget(self._build_estop_section())
        layout.addStretch(1)

    # --- construction -----------------------------------------------------

    def _build_status_section(self) -> QGroupBox:
        box = QGroupBox("Connection")
        layout = QVBoxLayout(box)

        row = QHBoxLayout()
        self._camera_status = QLabel("CAMERA")
        self._link_status = QLabel("ROBOT LINK")
        for label in (self._camera_status, self._link_status):
            label.setAlignment(Qt.AlignCenter)
            label.setFocusPolicy(Qt.NoFocus)
        row.addWidget(self._camera_status)
        row.addWidget(self._link_status)
        layout.addLayout(row)

        # Its own full-width row, not squeezed into the CAMERA/LINK row --
        # "ASSEMBLED (hold on fault)" needs room to read at a glance, not
        # get truncated fighting two other badges for width.
        self._safety_status = QLabel("SAFETY")
        self._safety_status.setAlignment(Qt.AlignCenter)
        self._safety_status.setFocusPolicy(Qt.NoFocus)
        layout.addWidget(self._safety_status)

        self._constants_warning_label = QLabel()
        self._constants_warning_label.setWordWrap(True)
        self._constants_warning_label.setFocusPolicy(Qt.NoFocus)
        self._constants_warning_label.hide()
        layout.addWidget(self._constants_warning_label)

        self.set_camera_connected(False)
        self.set_link_connected(False)
        self.set_safety_mode(None)
        self.set_constants_warning(None)
        return box

    def _build_telemetry_section(self) -> QGroupBox:
        box = QGroupBox("Telemetry")
        layout = QVBoxLayout(box)
        self._rtt_label = QLabel("RTT: n/a")
        self._last_applied_label = QLabel("last applied: n/a")
        self._last_applied_label.setWordWrap(True)
        self._gait_phase_label = QLabel("gait phase: n/a")
        self._fault_flags_label = QLabel("faults: none")
        for label in (
            self._rtt_label,
            self._last_applied_label,
            self._gait_phase_label,
            self._fault_flags_label,
        ):
            label.setFocusPolicy(Qt.NoFocus)
            layout.addWidget(label)
        return box

    def _build_keys_section(self) -> QGroupBox:
        box = QGroupBox("Keys")
        layout = QVBoxLayout(box)
        legend = QLabel(
            "WASD / Arrows: walk    Q / E: turn    Space: STOP\n"
            "Any manual key drops out of AUTO_TRACK."
        )
        legend.setFocusPolicy(Qt.NoFocus)
        layout.addWidget(legend)
        self._current_intent_label = QLabel("intent: stop")
        self._current_intent_label.setFocusPolicy(Qt.NoFocus)
        self._current_intent_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self._current_intent_label)
        return box

    def _build_sliders_section(self) -> QGroupBox:
        box = QGroupBox("Sliders")
        layout = QVBoxLayout(box)

        self._speed_slider, speed_row = self._make_slider(
            "speed", 0, 100, 30, self.speed_changed
        )
        self._height_slider, height_row = self._make_slider(
            "body height", 0, 100, 50, self.body_height_changed
        )
        self._pan_slider, pan_row = self._make_slider("pan", -90, 90, 0, self.pan_changed)
        self._tilt_slider, tilt_row = self._make_slider("tilt", -90, 90, 0, self.tilt_changed)

        for row in (speed_row, height_row, pan_row, tilt_row):
            layout.addLayout(row)
        return box

    def _make_slider(self, name: str, lo: int, hi: int, default: int, signal: Signal):
        slider = QSlider(Qt.Horizontal)
        slider.setRange(lo, hi)
        slider.setValue(default)
        slider.setFocusPolicy(Qt.NoFocus)  # never let a slider steal WASD/arrow/space

        value_label = QLabel(str(default))
        value_label.setFixedWidth(36)
        value_label.setFocusPolicy(Qt.NoFocus)

        def _on_change(value: int) -> None:
            value_label.setText(str(value))
            signal.emit(float(value))

        slider.valueChanged.connect(_on_change)

        row = QHBoxLayout()
        name_label = QLabel(name)
        name_label.setFixedWidth(80)
        name_label.setFocusPolicy(Qt.NoFocus)
        row.addWidget(name_label)
        row.addWidget(slider)
        row.addWidget(value_label)
        return slider, row

    def _build_mode_section(self) -> QGroupBox:
        box = QGroupBox("Mode")
        layout = QHBoxLayout(box)
        self._mode_button = QPushButton("MANUAL")
        self._mode_button.setCheckable(True)
        self._mode_button.setFocusPolicy(Qt.NoFocus)
        self._mode_button.toggled.connect(self.mode_toggle_requested.emit)
        layout.addWidget(self._mode_button)
        return box

    def _build_estop_section(self) -> QWidget:
        button = QPushButton("EMERGENCY STOP")
        button.setStyleSheet(_ESTOP_STYLE)
        button.setFocusPolicy(Qt.NoFocus)
        button.setMinimumHeight(64)
        button.clicked.connect(self.estop_clicked.emit)
        return button

    # --- state pushed in from MainWindow's tick ----------------------------

    def set_camera_connected(self, connected: bool) -> None:
        self._camera_status.setText("CAMERA: connected" if connected else "CAMERA: disconnected")
        self._camera_status.setStyleSheet(_CONNECTED_STYLE if connected else _DISCONNECTED_STYLE)

    def set_link_connected(self, connected: bool) -> None:
        self._link_status.setText("LINK: connected" if connected else "LINK: disconnected")
        self._link_status.setStyleSheet(_CONNECTED_STYLE if connected else _DISCONNECTED_STYLE)

    def set_safety_mode(self, robot_assembled: bool | None) -> None:
        """robot_assembled echoes firmware's compiled-in ROBOT_ASSEMBLED
        flag (Telemetry.robot_assembled) -- None before any telemetry has
        arrived. There is deliberately no "mismatch" warning state here:
        nothing in this system (no sensor, no cross-checkable PC-side
        expectation) can tell whether this value still matches physical
        reality, only what firmware last reported it compiled with. This
        badge exists so a human catches a stale flag by their own
        knowledge of whether the robot is actually assembled -- it is not
        making that judgment itself. See docs/protocol.md Section 9."""
        if robot_assembled is None:
            self._safety_status.setText("SAFETY: n/a")
            self._safety_status.setStyleSheet(_SAFETY_UNKNOWN_STYLE)
        elif robot_assembled:
            self._safety_status.setText("SAFETY: ASSEMBLED (hold on fault)")
            self._safety_status.setStyleSheet(_SAFETY_ASSEMBLED_STYLE)
        else:
            self._safety_status.setText("SAFETY: BENCH (release on fault)")
            self._safety_status.setStyleSheet(_SAFETY_BENCH_STYLE)

    def set_constants_warning(self, message: str | None) -> None:
        if message is None:
            self._constants_warning_label.hide()
            return
        self._constants_warning_label.setText(f"⚠ {message}")
        self._constants_warning_label.setStyleSheet(_WARNING_STYLE)
        self._constants_warning_label.show()

    def set_telemetry(self, telemetry: Telemetry | None, rtt_ms: float | None) -> None:
        if telemetry is None:
            self._rtt_label.setText("RTT: n/a")
            self._last_applied_label.setText("last applied: n/a")
            self._gait_phase_label.setText("gait phase: n/a")
            self._fault_flags_label.setText("faults: n/a")
            self.set_safety_mode(None)
            return

        self._rtt_label.setText(f"RTT: {rtt_ms:.0f}ms" if rtt_ms is not None else "RTT: n/a")
        self._last_applied_label.setText(f"last applied: {telemetry.last_applied!r}")
        phase = telemetry.gait_phase
        self._gait_phase_label.setText(
            f"gait phase: {phase:.2f}" if phase is not None else "gait phase: n/a"
        )
        self._fault_flags_label.setText(f"faults: {_fault_flags_text(telemetry.fault_flags)}")
        self.set_safety_mode(telemetry.robot_assembled)

    def set_current_intent(self, command: Command) -> None:
        self._current_intent_label.setText(f"intent: {command!r}")

    def set_mode(self, is_auto_track: bool) -> None:
        # Programmatic sync (e.g. MainWindow forcing MANUAL) must not
        # re-trigger mode_toggle_requested, which would try to force the
        # mode again and could loop.
        self._mode_button.blockSignals(True)
        self._mode_button.setChecked(is_auto_track)
        self._mode_button.setText("AUTO_TRACK" if is_auto_track else "MANUAL")
        self._mode_button.blockSignals(False)

    def current_speed(self) -> float:
        return float(self._speed_slider.value())

    def current_body_height(self) -> float:
        return float(self._height_slider.value())

    def current_pan(self) -> float:
        return float(self._pan_slider.value())

    def current_tilt(self) -> float:
        return float(self._tilt_slider.value())
