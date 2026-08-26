"""Bench Test tab: direct raw-pulse control of one PCA9685 channel for a
loose servo before it's mounted on a leg. No kinematics, no gait -- this is
the only place in the app that talks to hardware without going through
angle/IK math at all, which is why it's gated by its own bench_mode arm
switch (transport/protocol.py, docs/protocol.md Section 8), separate from
calibration_mode.

Two independent selectors on purpose:
  - board/channel: which physical PCA9685 pin the loose servo is wired to
    right now, for driving BenchPulseCommand. Purely about hardware wiring,
    no persistence.
  - servo_index: which future leg position this unit's findings (limits,
    health note) get recorded under, via RecordLimitCommand/
    BenchHealthNoteCommand. Purely about record-keeping, independent of
    which channel happens to be plugged in for this session.
See docs/protocol.md Section 8 / ANALYSIS.md for why these can't be the
same selector -- a loose servo doesn't have a leg position yet.

Stall protection (control/bench.py's DwellGuard) applies to Park/Manual/
range-finder holds, not to an active sweep -- a sweep is continuously
moving, not stalling, and is already bounded and abortable on its own.
"""

import time
import warnings

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from control.bench import BenchSafetyConfig, DwellGuard, SweepPlan
from transport.generated_constants import BENCH_ARM_TIMEOUT_S
from transport.link import RobotLink
from transport.protocol import (
    BENCH_PULSE_MAX_US,
    BENCH_PULSE_MIN_US,
    BenchHealthNoteCommand,
    BenchModeCommand,
    BenchPulseCommand,
    LimitBound,
    NEUTRAL_PULSE_US,
    PCA9685_BOARD_ADDRESSES,
    PCA9685_CHANNELS_PER_BOARD,
    ReadOffsetsCommand,
    RecordLimitCommand,
    ServoProfile,
    Telemetry,
)
from ui.servo_names import SERVO_NAMES

_SEND_TIMEOUT_S = 0.5
_ARMED_STYLE = "color: #ffb300; font-weight: bold;"
_DWELL_WARNING_STYLE = "background-color: #b71c1c; color: white; padding: 6px; font-weight: bold;"
_NOTE_STYLE = "color: #9e9e9e; font-style: italic;"


class BenchTab(QWidget):
    log_message = Signal(str)

    def __init__(
        self,
        link: RobotLink,
        dwell_timeout_s: float = 8.0,
        nudge_step_us: int = 10,
        default_sweep_min_us: int = 1400,
        default_sweep_max_us: int = 1600,
        default_sweep_duration_s: float = 4.0,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.link = link
        self._nudge_step_us = nudge_step_us

        if dwell_timeout_s >= BENCH_ARM_TIMEOUT_S:
            # Dwell protection only works if it fires before bench_mode
            # itself auto-disarms -- once disarmed, bench_pulse is rejected
            # and this tab can no longer send a park command at all. With
            # the packaged defaults (8s dwell, 60s bench arm) this can't
            # happen; only a deliberately misconfigured dwell_timeout_s
            # would trigger it.
            warnings.warn(
                f"bench dwell_timeout_s ({dwell_timeout_s}) >= BENCH_ARM_TIMEOUT_S "
                f"({BENCH_ARM_TIMEOUT_S}) -- bench mode could auto-disarm while a "
                "servo is held away from neutral, before the dwell guard ever fires.",
                stacklevel=2,
            )

        self._dwell_guard = DwellGuard(
            BenchSafetyConfig(neutral_pulse_us=NEUTRAL_PULSE_US, dwell_timeout_s=dwell_timeout_s)
        )
        self._armed_since: float | None = None
        self._current_pulse: int = NEUTRAL_PULSE_US
        self._active_sweep: SweepPlan | None = None
        self._sweep_started_at: float | None = None

        layout = QVBoxLayout(self)
        layout.addWidget(self._build_note_section())
        layout.addWidget(self._build_arm_section())
        layout.addWidget(self._build_channel_section())
        layout.addWidget(self._build_pulse_section())
        layout.addWidget(
            self._build_sweep_section(default_sweep_min_us, default_sweep_max_us, default_sweep_duration_s)
        )
        layout.addWidget(self._build_servo_section())
        layout.addWidget(self._build_dwell_warning())
        layout.addStretch(1)

        self._set_controls_enabled(False)

    # --- construction -----------------------------------------------------

    def _build_note_section(self) -> QLabel:
        label = QLabel(
            "Bench mode drives one PCA9685 channel directly, in microseconds. "
            "There is no leg and no IK here -- this pulse goes straight to the "
            "servo, unrelated to any leg or gait math."
        )
        label.setWordWrap(True)
        label.setStyleSheet(_NOTE_STYLE)
        label.setFocusPolicy(Qt.NoFocus)
        return label

    def _build_arm_section(self) -> QGroupBox:
        box = QGroupBox("Bench arm / disarm")
        layout = QHBoxLayout(box)

        self._armed_label = QLabel("DISARMED")
        self._armed_label.setFocusPolicy(Qt.NoFocus)
        self._countdown_label = QLabel("")
        self._countdown_label.setFocusPolicy(Qt.NoFocus)

        self._arm_button = QPushButton("Arm bench mode")
        self._arm_button.setFocusPolicy(Qt.NoFocus)
        self._arm_button.clicked.connect(self._on_arm_clicked)

        self._disarm_button = QPushButton("Disarm")
        self._disarm_button.setFocusPolicy(Qt.NoFocus)
        self._disarm_button.clicked.connect(self._on_disarm_clicked)
        self._disarm_button.setEnabled(False)

        layout.addWidget(self._armed_label)
        layout.addWidget(self._countdown_label)
        layout.addWidget(self._arm_button)
        layout.addWidget(self._disarm_button)
        return box

    def _build_channel_section(self) -> QGroupBox:
        box = QGroupBox("Channel (which physical pin the loose servo is on)")
        layout = QHBoxLayout(box)

        self._board_combo = QComboBox()
        self._board_combo.setFocusPolicy(Qt.NoFocus)
        for board in PCA9685_BOARD_ADDRESSES:
            self._board_combo.addItem(f"0x{board:02x}", board)
        self._board_combo.currentIndexChanged.connect(self._on_channel_changed)

        self._channel_spin = QSpinBox()
        self._channel_spin.setRange(0, PCA9685_CHANNELS_PER_BOARD - 1)
        self._channel_spin.setFocusPolicy(Qt.NoFocus)
        self._channel_spin.valueChanged.connect(self._on_channel_changed)

        layout.addWidget(QLabel("board"))
        layout.addWidget(self._board_combo)
        layout.addWidget(QLabel("channel"))
        layout.addWidget(self._channel_spin)
        return box

    def _build_pulse_section(self) -> QGroupBox:
        box = QGroupBox("Pulse control")
        layout = QVBoxLayout(box)

        self._park_button = QPushButton("Park at neutral (hold, no timeout)")
        self._park_button.setFocusPolicy(Qt.NoFocus)
        self._park_button.clicked.connect(self._on_park_clicked)
        layout.addWidget(self._park_button)

        pulse_row = QHBoxLayout()
        self._pulse_slider = QSlider(Qt.Horizontal)
        self._pulse_slider.setRange(BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US)
        self._pulse_slider.setValue(NEUTRAL_PULSE_US)
        self._pulse_slider.setFocusPolicy(Qt.NoFocus)
        self._pulse_slider.valueChanged.connect(self._on_slider_changed)
        self._pulse_value_label = QLabel(f"{NEUTRAL_PULSE_US}us")
        self._pulse_value_label.setFixedWidth(64)
        pulse_row.addWidget(QLabel("manual pulse"))
        pulse_row.addWidget(self._pulse_slider)
        pulse_row.addWidget(self._pulse_value_label)
        layout.addLayout(pulse_row)

        nudge_row = QHBoxLayout()
        minus_button = QPushButton(f"-{self._nudge_step_us}us")
        minus_button.setFocusPolicy(Qt.NoFocus)
        minus_button.clicked.connect(lambda: self._nudge(-self._nudge_step_us))
        plus_button = QPushButton(f"+{self._nudge_step_us}us")
        plus_button.setFocusPolicy(Qt.NoFocus)
        plus_button.clicked.connect(lambda: self._nudge(self._nudge_step_us))
        nudge_row.addWidget(QLabel("range finder nudge"))
        nudge_row.addWidget(minus_button)
        nudge_row.addWidget(plus_button)
        layout.addLayout(nudge_row)
        self._nudge_buttons = (minus_button, plus_button)

        return box

    def _build_sweep_section(self, default_min: int, default_max: int, default_duration: float) -> QGroupBox:
        box = QGroupBox("Sweep test (one pass, always abortable)")
        layout = QVBoxLayout(box)

        range_row = QHBoxLayout()
        self._sweep_min_spin = QSpinBox()
        self._sweep_min_spin.setRange(BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US)
        self._sweep_min_spin.setValue(default_min)
        self._sweep_min_spin.setFocusPolicy(Qt.NoFocus)
        self._sweep_max_spin = QSpinBox()
        self._sweep_max_spin.setRange(BENCH_PULSE_MIN_US, BENCH_PULSE_MAX_US)
        self._sweep_max_spin.setValue(default_max)
        self._sweep_max_spin.setFocusPolicy(Qt.NoFocus)
        self._sweep_duration_spin = QSpinBox()
        self._sweep_duration_spin.setRange(1, 60)
        self._sweep_duration_spin.setValue(int(default_duration))
        self._sweep_duration_spin.setFocusPolicy(Qt.NoFocus)
        range_row.addWidget(QLabel("min"))
        range_row.addWidget(self._sweep_min_spin)
        range_row.addWidget(QLabel("max"))
        range_row.addWidget(self._sweep_max_spin)
        range_row.addWidget(QLabel("seconds"))
        range_row.addWidget(self._sweep_duration_spin)
        layout.addLayout(range_row)
        note = QLabel(
            f"Defaults are conservative (default {default_min}-{default_max}us). "
            "Widen deliberately, not by starting wide."
        )
        note.setStyleSheet(_NOTE_STYLE)
        note.setWordWrap(True)
        layout.addWidget(note)

        button_row = QHBoxLayout()
        self._sweep_start_button = QPushButton("Start sweep")
        self._sweep_start_button.setFocusPolicy(Qt.NoFocus)
        self._sweep_start_button.clicked.connect(self._on_start_sweep_clicked)
        self._sweep_abort_button = QPushButton("ABORT SWEEP")
        self._sweep_abort_button.setFocusPolicy(Qt.NoFocus)
        self._sweep_abort_button.setStyleSheet("background-color: #b71c1c; color: white; font-weight: bold;")
        self._sweep_abort_button.clicked.connect(self._on_abort_sweep_clicked)
        self._sweep_abort_button.setEnabled(False)
        button_row.addWidget(self._sweep_start_button)
        button_row.addWidget(self._sweep_abort_button)
        layout.addLayout(button_row)

        return box

    def _build_servo_section(self) -> QGroupBox:
        box = QGroupBox("Record findings for this servo position")
        layout = QVBoxLayout(box)

        selector_row = QHBoxLayout()
        self._servo_combo = QComboBox()
        self._servo_combo.setFocusPolicy(Qt.NoFocus)
        for i, name in enumerate(SERVO_NAMES):
            self._servo_combo.addItem(f"{i}: {name}")
        self._servo_combo.currentIndexChanged.connect(self._on_servo_selected)
        selector_row.addWidget(QLabel("this unit will become"))
        selector_row.addWidget(self._servo_combo)
        layout.addLayout(selector_row)

        self._known_limits_label = QLabel("known limits: not bench-tested")
        self._known_limits_label.setFocusPolicy(Qt.NoFocus)
        layout.addWidget(self._known_limits_label)

        mark_row = QHBoxLayout()
        mark_min_button = QPushButton("Mark current pulse as MIN limit")
        mark_min_button.setFocusPolicy(Qt.NoFocus)
        mark_min_button.clicked.connect(lambda: self._on_mark_limit_clicked(LimitBound.MIN))
        mark_max_button = QPushButton("Mark current pulse as MAX limit")
        mark_max_button.setFocusPolicy(Qt.NoFocus)
        mark_max_button.clicked.connect(lambda: self._on_mark_limit_clicked(LimitBound.MAX))
        mark_row.addWidget(mark_min_button)
        mark_row.addWidget(mark_max_button)
        layout.addLayout(mark_row)
        self._mark_buttons = (mark_min_button, mark_max_button)

        note_row = QHBoxLayout()
        self._note_edit = QLineEdit()
        self._note_edit.setPlaceholderText("e.g. buzzes at low end, dead, fine")
        save_note_button = QPushButton("Save note")
        save_note_button.setFocusPolicy(Qt.NoFocus)
        save_note_button.clicked.connect(self._on_save_note_clicked)
        note_row.addWidget(QLabel("health note"))
        note_row.addWidget(self._note_edit)
        note_row.addWidget(save_note_button)
        layout.addLayout(note_row)

        self._on_servo_selected(0)  # populate the known-limits label initially
        return box

    def _build_dwell_warning(self) -> QLabel:
        label = QLabel()
        label.setWordWrap(True)
        label.hide()
        self._dwell_warning = label
        return label

    # --- periodic tick, called from MainWindow's central tick -------------

    def tick(self, telemetry: Telemetry | None) -> None:
        armed = telemetry.bench_armed if telemetry is not None else False
        now = time.monotonic()

        if armed and self._armed_since is not None:
            remaining = max(0.0, BENCH_ARM_TIMEOUT_S - (now - self._armed_since))
            self._countdown_label.setText(f"auto-disarms in {remaining:0.0f}s")
        else:
            self._countdown_label.setText("")

        if armed:
            self._armed_label.setText("ARMED")
            self._armed_label.setStyleSheet(_ARMED_STYLE)
        else:
            self._armed_label.setText("DISARMED")
            self._armed_label.setStyleSheet("")
            self._armed_since = None
            if self._active_sweep is not None:
                self._active_sweep = None
                self._sweep_started_at = None

        self._arm_button.setEnabled(not armed)
        self._disarm_button.setEnabled(armed)
        self._set_controls_enabled(armed)

        if self._active_sweep is not None:
            self._drive_sweep(now)
        elif armed:
            self._check_dwell(now)
        else:
            self._dwell_warning.hide()

    def emergency_stop(self) -> None:
        """Called by MainWindow's global e-stop (Space / STOP button) no
        matter which tab is visible -- the emergency stop is a whole-app
        concept, not scoped to motion. Aborts an active sweep and parks at
        neutral. Non-blocking (plain send(), not send_and_wait): an e-stop
        must never risk waiting on a round-trip. A no-op if bench mode
        isn't armed -- there's nothing bench-related happening to stop."""
        if self._active_sweep is not None:
            self._active_sweep = None
            self._sweep_started_at = None
            self._sweep_start_button.setEnabled(True)
            self._sweep_abort_button.setEnabled(False)
            self._set_manual_controls_enabled(True)
        if self._current_pulse != NEUTRAL_PULSE_US:
            self._send_pulse(NEUTRAL_PULSE_US)
            self.log_message.emit("emergency stop: bench parked at neutral")

    # --- arm/disarm ---------------------------------------------------

    def _on_arm_clicked(self) -> None:
        telemetry = self._send(BenchModeCommand(armed=True))
        if telemetry is not None and telemetry.ok:
            self._armed_since = time.monotonic()
            self.log_message.emit("bench mode armed")
        else:
            self.log_message.emit("failed to arm bench mode")

    def _on_disarm_clicked(self) -> None:
        if self._active_sweep is not None:
            self._on_abort_sweep_clicked()
        telemetry = self._send(BenchModeCommand(armed=False))
        if telemetry is not None and telemetry.ok:
            self.log_message.emit("bench mode disarmed")
        else:
            self.log_message.emit("failed to disarm bench mode")

    # --- channel / pulse -----------------------------------------------

    def _on_channel_changed(self) -> None:
        self._dwell_guard.reset()
        self._set_pulse_display(NEUTRAL_PULSE_US)
        self._current_pulse = NEUTRAL_PULSE_US
        self.log_message.emit(
            f"bench channel -> board {self._board_combo.currentText()} channel {self._channel_spin.value()}"
        )

    def _on_park_clicked(self) -> None:
        self._send_pulse(NEUTRAL_PULSE_US)
        self.log_message.emit("parked at neutral")

    def _on_slider_changed(self, value: int) -> None:
        self._pulse_value_label.setText(f"{value}us")
        if self._active_sweep is not None:
            return  # sweep owns the pulse output while running
        self._send_pulse(value)

    def _nudge(self, delta: int) -> None:
        new_pulse = max(BENCH_PULSE_MIN_US, min(BENCH_PULSE_MAX_US, self._current_pulse + delta))
        self._send_pulse(new_pulse)

    def _send_pulse(self, pulse_us: int) -> None:
        board = self._board_combo.currentData()
        channel = self._channel_spin.value()
        self.link.send(BenchPulseCommand(board=board, channel=channel, pulse_us=pulse_us))
        self._current_pulse = pulse_us
        self._set_pulse_display(pulse_us)

    def _set_pulse_display(self, pulse_us: int) -> None:
        self._pulse_slider.blockSignals(True)
        self._pulse_slider.setValue(pulse_us)
        self._pulse_slider.blockSignals(False)
        self._pulse_value_label.setText(f"{pulse_us}us")

    # --- sweep -----------------------------------------------------

    def _on_start_sweep_clicked(self) -> None:
        try:
            plan = SweepPlan(
                min_us=self._sweep_min_spin.value(),
                max_us=self._sweep_max_spin.value(),
                duration_s=float(self._sweep_duration_spin.value()),
            )
        except ValueError as exc:
            self.log_message.emit(f"cannot start sweep: {exc}")
            return

        self._active_sweep = plan
        self._sweep_started_at = time.monotonic()
        self._dwell_guard.reset()
        self._sweep_start_button.setEnabled(False)
        self._sweep_abort_button.setEnabled(True)
        self._set_manual_controls_enabled(False)
        self.log_message.emit(f"sweep started: {plan.min_us}-{plan.max_us}us over {plan.duration_s:.0f}s")

    def _on_abort_sweep_clicked(self) -> None:
        if self._active_sweep is None:
            return
        self._active_sweep = None
        self._sweep_started_at = None
        self._sweep_start_button.setEnabled(True)
        self._sweep_abort_button.setEnabled(False)
        self._set_manual_controls_enabled(True)
        self._send_pulse(NEUTRAL_PULSE_US)
        self.log_message.emit("sweep aborted, parked at neutral")

    def _drive_sweep(self, now: float) -> None:
        elapsed = now - self._sweep_started_at
        if self._active_sweep.is_complete(elapsed):
            self._send_pulse(NEUTRAL_PULSE_US)
            self._active_sweep = None
            self._sweep_started_at = None
            self._sweep_start_button.setEnabled(True)
            self._sweep_abort_button.setEnabled(False)
            self._set_manual_controls_enabled(True)
            self.log_message.emit("sweep complete, parked at neutral")
            return
        self._send_pulse(self._active_sweep.pulse_at(elapsed))

    # --- dwell / stall protection ---------------------------------------

    def _check_dwell(self, now: float) -> None:
        tripped = self._dwell_guard.observe(self._current_pulse, now)
        if tripped:
            self._send_pulse(NEUTRAL_PULSE_US)
            self._dwell_guard.reset()
            self._dwell_warning.hide()
            self.log_message.emit("dwell timeout: auto-returned to neutral")
            return

        remaining = self._dwell_guard.remaining_s(self._current_pulse, now)
        if remaining is None:
            self._dwell_warning.hide()
            return
        self._dwell_warning.setText(
            f"⚠ Away from neutral -- auto-returns in {remaining:0.0f}s unless you park it."
        )
        self._dwell_warning.setStyleSheet(_DWELL_WARNING_STYLE)
        self._dwell_warning.show()

    # --- per-servo recording ---------------------------------------------

    def _on_servo_selected(self, _index: int) -> None:
        telemetry = self._send(ReadOffsetsCommand())
        servo_index = self._servo_combo.currentIndex()
        if telemetry is None or not telemetry.ok or telemetry.profiles is None:
            self._known_limits_label.setText("known limits: (could not read from robot)")
            return
        profile = telemetry.profiles[servo_index]
        self._known_limits_label.setText(f"known limits: {_limits_text(profile)}")
        if profile.note:
            self._note_edit.setText(profile.note)
        else:
            self._note_edit.clear()

    def _on_mark_limit_clicked(self, bound: LimitBound) -> None:
        servo_index = self._servo_combo.currentIndex()
        telemetry = self._send(
            RecordLimitCommand(servo_index=servo_index, bound=bound, pulse_us=self._current_pulse)
        )
        if telemetry is None:
            self.log_message.emit("no response recording limit")
            return
        if not telemetry.ok:
            self.log_message.emit(f"rejected: {telemetry.error}")
            return
        self.log_message.emit(
            f"recorded {bound.value} limit for servo {servo_index} ({SERVO_NAMES[servo_index]}): "
            f"{self._current_pulse}us"
        )
        self._on_servo_selected(servo_index)  # refresh the known-limits label

    def _on_save_note_clicked(self) -> None:
        servo_index = self._servo_combo.currentIndex()
        note = self._note_edit.text()
        telemetry = self._send(BenchHealthNoteCommand(servo_index=servo_index, note=note))
        if telemetry is None or not telemetry.ok:
            self.log_message.emit(f"failed to save note: {telemetry.error if telemetry else 'no response'}")
            return
        self.log_message.emit(f"saved note for servo {servo_index} ({SERVO_NAMES[servo_index]}): {note!r}")

    # --- enable/disable ----------------------------------------------------

    def _set_controls_enabled(self, enabled: bool) -> None:
        self._park_button.setEnabled(enabled)
        self._set_manual_controls_enabled(enabled)
        self._sweep_start_button.setEnabled(enabled and self._active_sweep is None)

    def _set_manual_controls_enabled(self, enabled: bool) -> None:
        self._pulse_slider.setEnabled(enabled)
        for button in self._nudge_buttons:
            button.setEnabled(enabled)
        for button in self._mark_buttons:
            button.setEnabled(enabled)

    def _send(self, command) -> Telemetry | None:
        return self.link.send_and_wait(command, timeout_s=_SEND_TIMEOUT_S)


def _limits_text(profile: ServoProfile) -> str:
    if profile.min_pulse_us is None and profile.max_pulse_us is None:
        return "not bench-tested"
    lo = profile.min_pulse_us if profile.min_pulse_us is not None else "?"
    hi = profile.max_pulse_us if profile.max_pulse_us is not None else "?"
    return f"{lo}-{hi}us"
