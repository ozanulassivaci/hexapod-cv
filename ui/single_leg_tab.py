"""Test Leg tab: bring-up controls for the single physical test leg
(reference/ANALYSIS.md's bring-up plan) -- one leg clamped to a table
edge, three servos wired, before any of the other five legs exist.
Bench-gated (bench_mode), same shared arm switch and channel/servo_index
addressing split as the Bench Test tab (docs/protocol.md Section 8) --
this tab is built on the same wire commands (bench_pulse, bench_release,
record_limit), just organized per-joint with angle-space step buttons
and an IK solve instead of one raw pulse slider.

Local frame, not a specific leg's real body-relative position -- see
control/single_leg.py's TEST_LEG for the reasoning (this was the "should
this tab impersonate a leg index or work locally" design question,
resolved in favor of whichever makes a ruler check against the bare
physical leg require no mental arithmetic).

Joint mode and IK mode share one set of board/channel wiring (set up
once, used either way) and one "this test leg will become" leg selector,
which determines which three servo_index slots (leg*3 + 0/1/2) findings
get recorded under -- same reasoning as Bench Test's separate board/
channel vs. servo_index split, just for three channels instead of one.
"""

import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from control import gait_preview as gait
from control.manual_intent import MOVE_KEYS, TURN_KEYS, turn_intent, walk_intent
from control.single_leg import EXPLORATION_ORDER, TEST_LEG, deg_from_neutral, is_step_allowed
from robot.gait import (
    GAIT_ENVELOPE_COXA_MAX_DEG,
    GAIT_ENVELOPE_COXA_MIN_DEG,
    GAIT_ENVELOPE_FEMUR_MAX_DEG,
    GAIT_ENVELOPE_FEMUR_MIN_DEG,
    GAIT_ENVELOPE_TIBIA_MAX_DEG,
    GAIT_ENVELOPE_TIBIA_MIN_DEG,
)
from robot.gait import DEFAULT_BODY_HEIGHT
from robot.kinematics import (
    LEGS,
    SAFE_LIMIT_MARGIN_DEG,
    JointAngles,
    Point3,
    clamp_joint_angles,
    forward_kinematics,
    inverse_kinematics,
    to_servo_deg,
)
from transport.link import RobotLink
from transport.protocol import (
    BenchModeCommand,
    BenchPulseCommand,
    BenchReleaseCommand,
    LimitBound,
    NEUTRAL_PULSE_US,
    PCA9685_BOARD_ADDRESSES,
    PCA9685_CHANNELS_PER_BOARD,
    ReadOffsetsCommand,
    RecordLimitCommand,
    Telemetry,
    angle_to_pulse_us,
    neutral_pulse_us_for_servo,
    neutral_servo_deg_for_servo,
)
from transport.generated_constants import BENCH_ARM_TIMEOUT_S
from ui.single_leg_view import SingleLegView

_SEND_TIMEOUT_S = 0.5
_ARMED_STYLE = "color: #ffb300; font-weight: bold;"
_NOTE_STYLE = "color: #9e9e9e; font-style: italic;"
_RELEASE_STYLE = "background-color: #b71c1c; color: white; font-weight: bold;"
_COMMANDED_NOT_MEASURED_STYLE = (
    "background-color: #4a148c; color: white; padding: 6px; font-weight: bold;"
)
_JOINT_OFFSET = {"coxa": 0, "femur": 1, "tibia": 2}
_STEP_SIZES_DEG = (1.0, 2.0, 5.0, 10.0)  # smallest first -- see control/single_leg.py

# Already in deg-from-neutral frame (robot/gait.py's own constants --
# see control/single_leg.py's deg_from_neutral() for why that's the
# frame is_step_allowed() needs, and why tibia isn't just the raw
# envelope numbers).
_GAIT_ENVELOPE_DEG_FROM_NEUTRAL = {
    "coxa": (GAIT_ENVELOPE_COXA_MIN_DEG, GAIT_ENVELOPE_COXA_MAX_DEG),
    "femur": (GAIT_ENVELOPE_FEMUR_MIN_DEG, GAIT_ENVELOPE_FEMUR_MAX_DEG),
    "tibia": (GAIT_ENVELOPE_TIBIA_MIN_DEG, GAIT_ENVELOPE_TIBIA_MAX_DEG),
}


def _servo_deg_for_joint(joint: str, raw_deg: float) -> float:
    """to_servo_deg() is a pure, independent-per-field transform, so
    building a JointAngles with only this one field set and reading back
    the matching output field is exact -- avoids hand-duplicating the
    coxa/femur bipolar-90 vs. tibia zero-based-negated convention here."""
    angles = JointAngles(
        coxa_deg=raw_deg if joint == "coxa" else 0.0,
        femur_deg=raw_deg if joint == "femur" else 0.0,
        tibia_deg=raw_deg if joint == "tibia" else 0.0,
    )
    coxa_servo, femur_servo, tibia_servo = to_servo_deg(angles)
    return {"coxa": coxa_servo, "femur": femur_servo, "tibia": tibia_servo}[joint]


class SingleLegTab(QWidget):
    log_message = Signal(str)

    def __init__(self, link: RobotLink, parent=None) -> None:
        super().__init__(parent)
        self.link = link

        self._leg_index = 0
        self._current_deg = {"coxa": 0.0, "femur": 0.0, "tibia": 0.0}
        self._known_limits: dict[str, tuple[float | None, float | None]] = {
            "coxa": (None, None),
            "femur": (None, None),
            "tibia": (None, None),
        }
        self._armed_since: float | None = None
        self._last_profiles = None

        # --- gait preview state ---
        self._gait_phase = 0.0
        self._gait_angles = JointAngles(0.0, 0.0, 0.0)
        self._gait_live = False
        self._gait_override = False
        self._gait_last_tick: float | None = None
        self._gait_move_keys: set[int] = set()
        self._gait_turn_keys: set[int] = set()
        # Last integer pulse actually sent per (board, channel). Gait
        # commands three joints every tick; re-sending a pulse the
        # PCA9685 already holds changes nothing electrically, so skipping
        # it costs nothing and keeps the packet rate down. See the
        # deadband note in docs/GUI_GUIDE.md.
        self._last_sent_pulse: dict[tuple[int, int], int] = {}

        layout = QVBoxLayout(self)
        layout.addWidget(self._build_note_section())
        layout.addWidget(self._build_arm_section())
        layout.addWidget(self._build_leg_section())
        layout.addWidget(self._build_channel_section())
        layout.addWidget(self._build_release_section())

        self._view = SingleLegView(self._current_deg, self._known_limits)
        layout.addWidget(self._view, 1)
        layout.addWidget(self._build_commanded_not_measured_label())

        layout.addWidget(self._build_mode_toggle())

        self._stack = QStackedWidget()
        self._stack.addWidget(self._build_joint_mode())
        self._stack.addWidget(self._build_ik_mode())
        self._stack.addWidget(self._build_gait_mode())
        # Scrollable with a floor well below the stack's natural size
        # (three joints' worth of step/mark buttons adds up) -- without
        # this, that natural size becomes this tab's minimumSizeHint,
        # which then out-competes self._view for any extra room the
        # window has to give, exactly backwards from what should grow.
        # stretch=0 (the addWidget default below) keeps it at that floor
        # -- self._view is the one stretch=1 widget in this layout, so it
        # alone claims space beyond every section's own minimum.
        stack_scroll = QScrollArea()
        stack_scroll.setWidget(self._stack)
        stack_scroll.setWidgetResizable(True)
        stack_scroll.setFocusPolicy(Qt.NoFocus)
        stack_scroll.setMinimumHeight(220)
        # Ignored, not the QScrollArea default: with a Preferred vertical
        # policy the layout gives this its full natural sizeHint (the
        # stack's, ~480px) whenever the tab is tall enough to afford it,
        # leaving nothing for self._view's stretch to actually claim.
        # Ignored tells the layout "shrink me to my minimum first" --
        # self._view's stretch=1 then absorbs everything past that floor.
        stack_scroll.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Ignored)
        layout.addWidget(stack_scroll)

        # No trailing stretch here on purpose -- self._view (stretch=1
        # above) is the only widget in this layout that should claim
        # extra vertical space, so the 2D side/top views actually grow
        # when the window does instead of splitting the gain with blank
        # space down here.

        # Needed so WASD/QE reach this tab at all while previewing. Any
        # key this tab doesn't consume falls through to MainWindow, which
        # keeps the Operate tab's own handling working untouched.
        self.setFocusPolicy(Qt.StrongFocus)

        self._set_controls_enabled(False)
        self._refresh_known_limits()

    # --- keyboard (gait preview only) -------------------------------------

    def _gait_keys_active(self) -> bool:
        return self._stack.currentIndex() == 2 and self._gait_live

    def keyPressEvent(self, event) -> None:
        key = event.key()
        if self._gait_keys_active() and not event.isAutoRepeat():
            if key in MOVE_KEYS:
                self._gait_move_keys.add(key)
                return
            if key in TURN_KEYS:
                self._gait_turn_keys.add(key)
                return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        key = event.key()
        if self._gait_keys_active() and not event.isAutoRepeat():
            if key in MOVE_KEYS:
                self._gait_move_keys.discard(key)
                return
            if key in TURN_KEYS:
                self._gait_turn_keys.discard(key)
                return
        super().keyReleaseEvent(event)

    # --- construction -----------------------------------------------------

    def _build_note_section(self) -> QLabel:
        label = QLabel(
            "One leg, clamped to a table edge, three servos wired -- before "
            "any of the other five legs exist. Shares bench mode's arm "
            "switch with the Bench Test tab (arming either one arms both)."
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

    def _build_leg_section(self) -> QGroupBox:
        box = QGroupBox("This test leg will become")
        layout = QHBoxLayout(box)
        self._leg_combo = QComboBox()
        self._leg_combo.setFocusPolicy(Qt.NoFocus)
        for i, leg in enumerate(LEGS):
            self._leg_combo.addItem(f"{i}: {leg.name}")
        self._leg_combo.currentIndexChanged.connect(self._on_leg_changed)
        layout.addWidget(QLabel("leg position"))
        layout.addWidget(self._leg_combo)
        note = QLabel("determines which 3 servo_index slots (coxa/femur/tibia) findings are recorded under")
        note.setStyleSheet(_NOTE_STYLE)
        layout.addWidget(note)
        return box

    def _build_channel_section(self) -> QGroupBox:
        box = QGroupBox("Channel wiring (which physical PCA9685 pins the three servos are on)")
        layout = QVBoxLayout(box)
        self._board_combo: dict[str, QComboBox] = {}
        self._channel_spin: dict[str, QSpinBox] = {}
        for joint in ("coxa", "femur", "tibia"):
            row = QHBoxLayout()
            row.addWidget(QLabel(joint))
            board_combo = QComboBox()
            board_combo.setFocusPolicy(Qt.NoFocus)
            for board in PCA9685_BOARD_ADDRESSES:
                board_combo.addItem(f"0x{board:02x}", board)
            channel_spin = QSpinBox()
            channel_spin.setRange(0, PCA9685_CHANNELS_PER_BOARD - 1)
            channel_spin.setFocusPolicy(Qt.NoFocus)
            row.addWidget(QLabel("board"))
            row.addWidget(board_combo)
            row.addWidget(QLabel("channel"))
            row.addWidget(channel_spin)
            layout.addLayout(row)
            self._board_combo[joint] = board_combo
            self._channel_spin[joint] = channel_spin
        return box

    def _build_release_section(self) -> QWidget:
        button = QPushButton("RELEASE LEG (instant, all three joints go limp)")
        button.setFocusPolicy(Qt.NoFocus)
        button.setStyleSheet(_RELEASE_STYLE)
        button.setMinimumHeight(40)
        button.clicked.connect(self._on_release_clicked)
        self._release_button = button
        return button

    def _build_mode_toggle(self) -> QWidget:
        row = QHBoxLayout()
        container = QWidget()
        container.setLayout(row)
        self._joint_mode_button = QPushButton("Joint mode")
        self._joint_mode_button.setFocusPolicy(Qt.NoFocus)
        self._joint_mode_button.setCheckable(True)
        self._joint_mode_button.setChecked(True)
        self._joint_mode_button.clicked.connect(lambda: self._set_mode(0))
        self._ik_mode_button = QPushButton("IK mode")
        self._ik_mode_button.setFocusPolicy(Qt.NoFocus)
        self._ik_mode_button.setCheckable(True)
        self._ik_mode_button.clicked.connect(lambda: self._set_mode(1))
        self._gait_mode_button = QPushButton("Gait preview")
        self._gait_mode_button.setFocusPolicy(Qt.NoFocus)
        self._gait_mode_button.setCheckable(True)
        self._gait_mode_button.clicked.connect(lambda: self._set_mode(2))
        row.addWidget(self._joint_mode_button)
        row.addWidget(self._ik_mode_button)
        row.addWidget(self._gait_mode_button)
        return container

    def _set_mode(self, index: int) -> None:
        # Leaving gait preview always stops live motion. Switching tabs
        # or modes must never leave the leg walking behind your back.
        if index != 2:
            self._stop_gait_live()
        self._stack.setCurrentIndex(index)
        self._joint_mode_button.setChecked(index == 0)
        self._ik_mode_button.setChecked(index == 1)
        self._gait_mode_button.setChecked(index == 2)
        self._view.set_gait_overlay(None, 0.0, index == 2, False)

    def _build_joint_mode(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)

        order_text = "  ->  ".join(f"{joint} ({reason})" for joint, reason in EXPLORATION_ORDER)
        order_label = QLabel(f"Suggested order: {order_text}")
        order_label.setWordWrap(True)
        order_label.setStyleSheet(_NOTE_STYLE)
        layout.addWidget(order_label)

        mark_note = QLabel(
            "The value to record with Mark MIN/MAX is 3-5 degrees back from where "
            "binding starts, not the binding point itself."
        )
        mark_note.setWordWrap(True)
        mark_note.setStyleSheet(_NOTE_STYLE)
        layout.addWidget(mark_note)

        step_note = QLabel(
            "Larger step buttons work immediately inside the gait envelope -- "
            "the range this joint is commanded across during normal walking, "
            "already known safe. They drop to 1-degree-only once a step would "
            "land past that envelope; that's the real unknown-territory edge "
            "this rule exists to slow you down for. Marking a limit extends "
            "the safe range further, the same way."
        )
        step_note.setWordWrap(True)
        step_note.setStyleSheet(_NOTE_STYLE)
        layout.addWidget(step_note)

        self._angle_label: dict[str, QLabel] = {}
        self._limits_label: dict[str, QLabel] = {}
        self._step_buttons: dict[str, dict[float, QPushButton]] = {}

        for joint in ("coxa", "femur", "tibia"):
            box = QGroupBox(joint)
            box_layout = QVBoxLayout(box)

            angle_label = QLabel("0.0 deg from neutral")
            angle_label.setFocusPolicy(Qt.NoFocus)
            box_layout.addWidget(angle_label)
            self._angle_label[joint] = angle_label

            step_row = QHBoxLayout()
            buttons: dict[float, QPushButton] = {}
            for step in reversed(_STEP_SIZES_DEG):
                b = QPushButton(f"-{step:g}")
                b.setFocusPolicy(Qt.NoFocus)
                b.clicked.connect(lambda _checked=False, j=joint, s=step: self._on_step_clicked(j, -s))
                step_row.addWidget(b)
                buttons[-step] = b
            for step in _STEP_SIZES_DEG:
                b = QPushButton(f"+{step:g}")
                b.setFocusPolicy(Qt.NoFocus)
                b.clicked.connect(lambda _checked=False, j=joint, s=step: self._on_step_clicked(j, s))
                step_row.addWidget(b)
                buttons[step] = b
            box_layout.addLayout(step_row)
            self._step_buttons[joint] = buttons

            mark_row = QHBoxLayout()
            mark_min = QPushButton("Mark current as MIN")
            mark_min.setFocusPolicy(Qt.NoFocus)
            mark_min.clicked.connect(lambda _checked=False, j=joint: self._on_mark_clicked(j, LimitBound.MIN))
            mark_max = QPushButton("Mark current as MAX")
            mark_max.setFocusPolicy(Qt.NoFocus)
            mark_max.clicked.connect(lambda _checked=False, j=joint: self._on_mark_clicked(j, LimitBound.MAX))
            mark_row.addWidget(mark_min)
            mark_row.addWidget(mark_max)
            box_layout.addLayout(mark_row)

            limits_label = QLabel("known limits: not bench-tested")
            limits_label.setFocusPolicy(Qt.NoFocus)
            box_layout.addWidget(limits_label)
            self._limits_label[joint] = limits_label

            layout.addWidget(box)

        return container

    def _build_ik_mode(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)

        ruler_note = QLabel(
            "X/Y/Z are in this leg's own local frame, not the robot body frame: "
            "measure with a ruler from the coxa's own rotation axis -- X forward "
            "along the coxa's zero direction (straight out from the mount, the "
            "direction it points at 0 deg / 90 servo degrees), Y sideways, Z down."
        )
        ruler_note.setWordWrap(True)
        ruler_note.setStyleSheet(_NOTE_STYLE)
        layout.addWidget(ruler_note)

        safety_note = QLabel(
            "IK mode jumps directly to a computed target -- it does not go through "
            "Joint mode's step-size safety rule. Use Joint mode first to find and "
            "mark real limits on all three joints; only the limits already marked "
            "are enforced here (silently rejected if violated, same as a bench_pulse)."
        )
        safety_note.setWordWrap(True)
        safety_note.setStyleSheet(_NOTE_STYLE)
        layout.addWidget(safety_note)

        target_row = QHBoxLayout()
        self._target_x = QDoubleSpinBox()
        self._target_y = QDoubleSpinBox()
        self._target_z = QDoubleSpinBox()
        for spin, label, default in (
            (self._target_x, "X (mm)", 150.0),
            (self._target_y, "Y (mm)", 0.0),
            (self._target_z, "Z (mm)", -80.0),
        ):
            spin.setRange(-500.0, 500.0)
            spin.setDecimals(1)
            spin.setValue(default)
            spin.setFocusPolicy(Qt.NoFocus)
            target_row.addWidget(QLabel(label))
            target_row.addWidget(spin)
        layout.addLayout(target_row)

        self._drive_ik_button = QPushButton("Drive to this target")
        self._drive_ik_button.setFocusPolicy(Qt.NoFocus)
        self._drive_ik_button.clicked.connect(self._on_drive_ik_clicked)
        layout.addWidget(self._drive_ik_button)

        self._ik_angles_label = QLabel("")
        self._ik_angles_label.setFocusPolicy(Qt.NoFocus)
        self._ik_angles_label.setWordWrap(True)
        layout.addWidget(self._ik_angles_label)

        self._ik_achieved_label = QLabel("")
        self._ik_achieved_label.setFocusPolicy(Qt.NoFocus)
        self._ik_achieved_label.setWordWrap(True)
        layout.addWidget(self._ik_achieved_label)

        return container

    def _build_commanded_not_measured_label(self) -> QLabel:
        label = QLabel("COMMANDED POSE -- NOT MEASURED. No position feedback exists anywhere in this system.")
        label.setStyleSheet(_COMMANDED_NOT_MEASURED_STYLE)
        label.setAlignment(Qt.AlignCenter)
        label.setFocusPolicy(Qt.NoFocus)
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

        self._arm_button.setEnabled(not armed)
        self._disarm_button.setEnabled(armed)

        if self._stack.currentIndex() == 2:
            self._gait_tick()
        self._gait_start_button.setEnabled(self._enabled_base and not self._gait_live)
        self._gait_stop_button.setEnabled(self._gait_live)
        self._set_controls_enabled(armed)
        self._update_step_button_states()
        self._view.tick()

    def emergency_stop(self) -> None:
        """Called by MainWindow's global e-stop no matter which tab is
        visible. Releases all three joints -- the same instant, always-
        reachable action as the RELEASE LEG button, not a park (park
        still commands and holds a pulse)."""
        self._release_all()

    # --- arm/disarm ---------------------------------------------------

    def _on_arm_clicked(self) -> None:
        telemetry = self._send(BenchModeCommand(armed=True))
        if telemetry is not None and telemetry.ok:
            self._armed_since = time.monotonic()
            self.log_message.emit("bench mode armed (test leg)")
        else:
            self.log_message.emit("failed to arm bench mode")

    def _on_disarm_clicked(self) -> None:
        telemetry = self._send(BenchModeCommand(armed=False))
        if telemetry is not None and telemetry.ok:
            self.log_message.emit("bench mode disarmed")
        else:
            self.log_message.emit("failed to disarm bench mode")

    # --- leg / servo_index --------------------------------------------

    def _on_leg_changed(self, index: int) -> None:
        # A different leg means different servos, a different phase
        # offset and a different geometry -- none of which the current
        # pre-flight result or live motion applies to.
        self._stop_gait_live()
        self._leg_index = index
        self._refresh_known_limits()

    def _servo_index(self, joint: str) -> int:
        return self._leg_index * 3 + _JOINT_OFFSET[joint]

    def _refresh_known_limits(self) -> None:
        telemetry = self._send(ReadOffsetsCommand())
        if telemetry is None or not telemetry.ok or telemetry.profiles is None:
            return
        self._last_profiles = telemetry.profiles
        for joint in ("coxa", "femur", "tibia"):
            profile = telemetry.profiles[self._servo_index(joint)]
            self._known_limits[joint] = (profile.min_deg_from_neutral, profile.max_deg_from_neutral)
            self._limits_label[joint].setText(f"known limits: {_limits_text(profile)}")
        self._refresh_gait_preflight()
        self._view.tick()

    # --- release -----------------------------------------------------

    def _on_release_clicked(self) -> None:
        self._release_all()
        self.log_message.emit("test leg released (all three joints limp)")

    def _release_all(self) -> None:
        # Releasing stops gait outright -- it is the panic control, and
        # a live preview re-commanding pulses would fight it.
        self._stop_gait_live()
        # The channels are about to be driven to no pulse at all, so
        # whatever was last sent no longer describes them; clearing this
        # stops the dedupe above from skipping the next real command.
        self._last_sent_pulse.clear()
        for joint in ("coxa", "femur", "tibia"):
            board = self._board_combo[joint].currentData()
            channel = self._channel_spin[joint].value()
            self.link.send(BenchReleaseCommand(board=board, channel=channel))

    # --- joint mode -----------------------------------------------------

    def _on_step_clicked(self, joint: str, step_deg: float) -> None:
        direction = 1 if step_deg > 0 else -1
        envelope_min, envelope_max = _GAIT_ENVELOPE_DEG_FROM_NEUTRAL[joint]
        allowed = is_step_allowed(
            deg_from_neutral(joint, self._current_deg[joint]),
            abs(step_deg),
            int(deg_from_neutral(joint, direction)),
            self._known_limits[joint][0],
            self._known_limits[joint][1],
            smallest_step_deg=_STEP_SIZES_DEG[0],
            envelope_min_deg=envelope_min,
            envelope_max_deg=envelope_max,
        )
        if not allowed:
            self.log_message.emit(f"{joint}: step refused, past both the marked-safe and gait-envelope range")
            return
        new_deg = self._current_deg[joint] + step_deg
        self._send_joint_angle(joint, new_deg)

    def _send_joint_angle(self, joint: str, raw_deg: float) -> None:
        servo_deg = _servo_deg_for_joint(joint, raw_deg)
        servo_index = self._servo_index(joint)
        pulse_us = angle_to_pulse_us(
            servo_deg, neutral_servo_deg_for_servo(servo_index), neutral_pulse_us_for_servo(servo_index), 1, 0
        )
        board = self._board_combo[joint].currentData()
        channel = self._channel_spin[joint].value()
        # Skip a pulse the PCA9685 is already holding. Rewriting an
        # identical value produces an identical waveform, so this changes
        # nothing the servo can see -- but gait preview commands three
        # joints every tick, and most ticks don't move every joint, so
        # this keeps the packet rate near what's actually changing. It
        # also removes any chance of dither from angle_to_pulse_us()'s
        # rounding flipping between two adjacent integers while a joint
        # sits at its goal.
        if self._last_sent_pulse.get((board, channel)) != pulse_us:
            self.link.send(
                BenchPulseCommand(board=board, channel=channel, pulse_us=pulse_us, servo_index=servo_index)
            )
            self._last_sent_pulse[(board, channel)] = pulse_us
        self._current_deg[joint] = raw_deg
        self._angle_label[joint].setText(f"{raw_deg:+.1f} deg from neutral")
        self._view.tick()  # live stick figure follows every commanded step immediately, not on the next tick()

    def _on_mark_clicked(self, joint: str, bound: LimitBound) -> None:
        servo_index = self._servo_index(joint)
        # Nothing echoes back "what pulse is this channel actually
        # holding right now" (bench_pulse isn't covered by
        # Telemetry.last_applied, docs/protocol.md Section 3), so this
        # tab's own tracked _current_deg -- kept in sync by
        # _send_joint_angle on every step/IK-drive -- is the only
        # available ground truth, same trust model as Bench Test tab's
        # _current_pulse.
        pulse_us = angle_to_pulse_us(
            _servo_deg_for_joint(joint, self._current_deg[joint]),
            neutral_servo_deg_for_servo(servo_index),
            neutral_pulse_us_for_servo(servo_index),
            1,
            0,
        )
        telemetry = self._send(RecordLimitCommand(servo_index=servo_index, bound=bound, pulse_us=pulse_us))
        if telemetry is None:
            self.log_message.emit(f"{joint}: no response recording limit")
            return
        if not telemetry.ok:
            self.log_message.emit(f"{joint}: rejected: {telemetry.error}")
            return
        self.log_message.emit(f"{joint} (servo {servo_index}): recorded {bound.value} limit")
        self._refresh_known_limits()

    def _update_step_button_states(self) -> None:
        for joint, buttons in self._step_buttons.items():
            marked_min, marked_max = self._known_limits[joint]
            envelope_min, envelope_max = _GAIT_ENVELOPE_DEG_FROM_NEUTRAL[joint]
            current = deg_from_neutral(joint, self._current_deg[joint])
            for signed_step, button in buttons.items():
                direction = 1 if signed_step > 0 else -1
                allowed = is_step_allowed(
                    current,
                    abs(signed_step),
                    int(deg_from_neutral(joint, direction)),
                    marked_min,
                    marked_max,
                    smallest_step_deg=_STEP_SIZES_DEG[0],
                    envelope_min_deg=envelope_min,
                    envelope_max_deg=envelope_max,
                )
                button.setEnabled(allowed and self._enabled_base)

    # --- IK mode -----------------------------------------------------

    def _on_drive_ik_clicked(self) -> None:
        target = Point3(x=self._target_x.value(), y=self._target_y.value(), z=self._target_z.value())
        raw_angles = inverse_kinematics(target, TEST_LEG)
        clamped = clamp_joint_angles(raw_angles)
        achieved = forward_kinematics(clamped, TEST_LEG)

        self._ik_angles_label.setText(
            f"joint angles (raw, from neutral): coxa {clamped.coxa_deg:+.1f} deg, "
            f"femur {clamped.femur_deg:+.1f} deg, tibia {clamped.tibia_deg:+.1f} deg"
        )
        clipped = (
            abs(achieved.x - target.x) > 0.5 or abs(achieved.y - target.y) > 0.5 or abs(achieved.z - target.z) > 0.5
        )
        achieved_text = (
            f"achieved foot position: ({achieved.x:.1f}, {achieved.y:.1f}, {achieved.z:.1f}) mm"
        )
        if clipped:
            achieved_text += " -- CLIPPED, differs from requested target (out of reach or past a joint bound)"
        self._ik_achieved_label.setText(achieved_text)

        for joint, raw_deg in (
            ("coxa", clamped.coxa_deg),
            ("femur", clamped.femur_deg),
            ("tibia", clamped.tibia_deg),
        ):
            self._send_joint_angle(joint, raw_deg)
            self._angle_label[joint].setText(f"{raw_deg:+.1f} deg from neutral")

    # --- enable/disable ----------------------------------------------------


    # --- gait preview mode -------------------------------------------------

    def _build_gait_mode(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)

        note = QLabel(
            "Drives this leg through the REAL gait cycle (robot/gait.py, the same "
            "engine the assembled robot walks with) at the phase offset of the leg "
            "position selected above. Clamp the leg with the foot hanging free "
            "before running this -- see docs/HOW_TO_USE.md."
        )
        note.setWordWrap(True)
        note.setStyleSheet(_NOTE_STYLE)
        layout.addWidget(note)

        # Pre-flight
        pre_box = QGroupBox("Pre-flight: does gait fit inside your marked limits?")
        pre_layout = QVBoxLayout(pre_box)
        self._gait_preflight_label = QLabel("not checked yet")
        self._gait_preflight_label.setWordWrap(True)
        self._gait_preflight_label.setFocusPolicy(Qt.NoFocus)
        pre_layout.addWidget(self._gait_preflight_label)
        self._gait_override_check = QCheckBox(
            "Override: run anyway, past what I have marked (I accept the risk)"
        )
        self._gait_override_check.setFocusPolicy(Qt.NoFocus)
        self._gait_override_check.toggled.connect(self._on_gait_override_toggled)
        pre_layout.addWidget(self._gait_override_check)
        layout.addWidget(pre_box)

        # Intent
        intent_box = QGroupBox("Walk intent (same keys as the Operate tab)")
        intent_layout = QVBoxLayout(intent_box)
        keys_note = QLabel("WASD / arrows: walk    Q / E: turn    (click the leg view first if keys do nothing)")
        keys_note.setStyleSheet(_NOTE_STYLE)
        intent_layout.addWidget(keys_note)
        self._gait_intent_label = QLabel("intent: idle")
        self._gait_intent_label.setStyleSheet("font-weight: bold;")
        self._gait_intent_label.setFocusPolicy(Qt.NoFocus)
        intent_layout.addWidget(self._gait_intent_label)

        self._gait_speed_slider, speed_row = self._build_gait_slider("gait speed", 0, 100, 40)
        intent_layout.addLayout(speed_row)
        self._gait_height_slider, height_row = self._build_gait_slider(
            "body height", 0, 100, int(DEFAULT_BODY_HEIGHT)
        )
        intent_layout.addLayout(height_row)
        self._gait_slow_slider, slow_row = self._build_gait_slider(
            "slow motion %", int(gait.MIN_SLOW_MOTION * 100), int(gait.MAX_SLOW_MOTION * 100),
            int(gait.DEFAULT_SLOW_MOTION * 100),
        )
        intent_layout.addLayout(slow_row)
        slow_note = QLabel(
            "Slow motion scales the whole preview -- phase rate and the per-joint "
            "slew bound together -- independently of gait speed above. Gait speed "
            "changes what the gait *is* (stride length and phase rate); slow motion "
            "only changes how fast you watch it."
        )
        slow_note.setWordWrap(True)
        slow_note.setStyleSheet(_NOTE_STYLE)
        intent_layout.addWidget(slow_note)
        layout.addWidget(intent_box)

        # Phase
        phase_box = QGroupBox("Gait cycle")
        phase_layout = QVBoxLayout(phase_box)
        self._gait_phase_label = QLabel("phase 0.000    STANCE")
        self._gait_phase_label.setStyleSheet("font-weight: bold;")
        self._gait_phase_label.setFocusPolicy(Qt.NoFocus)
        phase_layout.addWidget(self._gait_phase_label)

        step_row = QHBoxLayout()
        for step in reversed(gait.PHASE_STEPS):
            b = QPushButton(f"-{step:g}")
            b.setFocusPolicy(Qt.NoFocus)
            b.clicked.connect(lambda _c=False, d=-step: self._on_gait_phase_step(d))
            step_row.addWidget(b)
        for step in gait.PHASE_STEPS:
            b = QPushButton(f"+{step:g}")
            b.setFocusPolicy(Qt.NoFocus)
            b.clicked.connect(lambda _c=False, d=step: self._on_gait_phase_step(d))
            step_row.addWidget(b)
        phase_layout.addLayout(step_row)

        self._gait_zoom_check = QCheckBox("Zoom the side view to the foot path (the arc is tiny at full-leg scale)")
        self._gait_zoom_check.setFocusPolicy(Qt.NoFocus)
        self._gait_zoom_check.setChecked(True)
        phase_layout.addWidget(self._gait_zoom_check)

        self._gait_phase_slider = QSlider(Qt.Horizontal)
        self._gait_phase_slider.setFocusPolicy(Qt.NoFocus)
        self._gait_phase_slider.setRange(0, 999)
        self._gait_phase_slider.valueChanged.connect(self._on_gait_phase_slider)
        phase_layout.addWidget(self._gait_phase_slider)

        run_row = QHBoxLayout()
        self._gait_start_button = QPushButton("Start live gait")
        self._gait_start_button.setFocusPolicy(Qt.NoFocus)
        self._gait_start_button.clicked.connect(self._on_gait_start)
        self._gait_stop_button = QPushButton("Stop (hold at phase)")
        self._gait_stop_button.setFocusPolicy(Qt.NoFocus)
        self._gait_stop_button.clicked.connect(lambda: self._stop_gait_live())
        run_row.addWidget(self._gait_start_button)
        run_row.addWidget(self._gait_stop_button)
        phase_layout.addLayout(run_row)
        layout.addWidget(phase_box)

        self._gait_angles_label = QLabel("commanded: coxa 0.0  femur 0.0  tibia 0.0 (deg from neutral)")
        self._gait_angles_label.setFocusPolicy(Qt.NoFocus)
        layout.addWidget(self._gait_angles_label)
        return container

    def _build_gait_slider(self, name: str, lo: int, hi: int, initial: int):
        row = QHBoxLayout()
        label = QLabel(name)
        label.setFixedWidth(110)
        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.NoFocus)
        slider.setRange(lo, hi)
        slider.setValue(initial)
        value_label = QLabel(str(initial))
        value_label.setFixedWidth(36)
        slider.valueChanged.connect(lambda v: value_label.setText(str(v)))
        row.addWidget(label)
        row.addWidget(slider, 1)
        row.addWidget(value_label)
        return slider, row

    # --- gait preview behaviour -------------------------------------------

    def _gait_intent(self) -> tuple[float, float, float, float, float]:
        """(vx, vy, speed, rotation, body_height) from the live key state
        and sliders -- walk_intent/turn_intent are the same functions the
        Operate tab's own WASD handling uses, so identical keys really do
        produce identical intent rather than merely similar intent."""
        vx, vy = walk_intent(self._gait_move_keys)
        rotation = turn_intent(self._gait_turn_keys)
        if vx == 0.0 and vy == 0.0 and rotation != 0.0:
            pass  # turning in place is a valid intent on its own
        speed = float(self._gait_speed_slider.value())
        body_height = float(self._gait_height_slider.value())
        return vx, vy, speed, rotation, body_height

    def _gait_slow_motion(self) -> float:
        return self._gait_slow_slider.value() / 100.0

    def _refresh_gait_preflight(self) -> "gait.PreflightResult":
        profiles = self._last_profiles
        result = gait.preflight(self._leg_index, profiles)
        if result.fits:
            self._gait_preflight_label.setText(
                "OK -- gait's envelope fits inside every marked limit for this leg, "
                f"with {int(SAFE_LIMIT_MARGIN_DEG)} deg of safe margin to spare."
            )
            self._gait_preflight_label.setStyleSheet("color: #2e7d32;")
        elif profiles is None:
            self._gait_preflight_label.setText(
                "BLOCKED -- no servo profiles read back yet. Arm bench mode so limits can be read."
            )
            self._gait_preflight_label.setStyleSheet(_RELEASE_STYLE)
        else:
            lines = "\n".join(f"  - {line}" for line in result.summary_lines())
            self._gait_preflight_label.setText(
                "BLOCKED -- gait would drive this leg past what you have verified:\n"
                f"{lines}\n"
                "Mark the missing limits in Joint mode, or tick Override to run anyway."
            )
            self._gait_preflight_label.setStyleSheet(_RELEASE_STYLE)
        return result

    def _on_gait_override_toggled(self, checked: bool) -> None:
        self._gait_override = checked
        if checked:
            self.log_message.emit(
                "gait preview: pre-flight OVERRIDDEN -- gait may drive this leg past marked limits. "
                "Firmware still enforces whatever limits ARE marked (GatedServoDriver), but "
                "an unmarked direction has nothing to enforce."
            )

    def _on_gait_start(self) -> None:
        result = self._refresh_gait_preflight()
        if not result.fits and not self._gait_override:
            self.log_message.emit("gait preview: refused to start -- pre-flight failed, see the Gait preview panel")
            return
        self._gait_live = True
        self._gait_last_tick = None
        self.setFocus()  # so WASD/QE land here rather than on the Operate tab's handler
        self.log_message.emit(
            f"gait preview: live at {int(self._gait_slow_motion() * 100)}% speed -- "
            "RELEASE LEG stops it instantly"
        )

    def _stop_gait_live(self) -> None:
        if self._gait_live:
            self.log_message.emit("gait preview: stopped, holding at current phase")
        self._gait_live = False
        self._gait_move_keys.clear()
        self._gait_turn_keys.clear()

    def _on_gait_phase_step(self, delta: float) -> None:
        self._gait_phase = (self._gait_phase + delta) % 1.0
        self._sync_gait_phase_slider()

    def _on_gait_phase_slider(self, value: int) -> None:
        if not self._gait_live:
            self._gait_phase = value / 1000.0

    def _sync_gait_phase_slider(self) -> None:
        self._gait_phase_slider.blockSignals(True)
        self._gait_phase_slider.setValue(int(self._gait_phase * 1000) % 1000)
        self._gait_phase_slider.blockSignals(False)

    def _gait_tick(self) -> None:
        """One preview tick: advance phase if live, compute this leg's
        real gait goal at that phase, slew toward it, command it. Both
        step-through and live share this path -- the only difference is
        whether phase moves on its own."""
        now = time.monotonic()
        dt = 0.0 if self._gait_last_tick is None else min(0.25, now - self._gait_last_tick)
        self._gait_last_tick = now

        vx, vy, speed, rotation, body_height = self._gait_intent()
        slow = self._gait_slow_motion()

        if self._gait_live:
            self._gait_phase = gait.advance_phase(self._gait_phase, dt, speed, slow)
            self._sync_gait_phase_slider()

        goal = gait.goal_angles(self._leg_index, self._gait_phase, vx, vy, speed, rotation, body_height)
        self._gait_angles = gait.slew_toward(self._gait_angles, goal, dt, slow)

        if self._enabled_base:
            for joint, raw in (
                ("coxa", self._gait_angles.coxa_deg),
                ("femur", self._gait_angles.femur_deg),
                ("tibia", self._gait_angles.tibia_deg),
            ):
                self._send_joint_angle(joint, raw)

        stance = gait.stance_label(self._leg_index, self._gait_phase)
        self._gait_phase_label.setText(f"phase {self._gait_phase:0.3f}    {stance}")
        self._gait_phase_label.setStyleSheet(
            "font-weight: bold; color: #ef6c00;" if stance == "SWING" else "font-weight: bold; color: #1565c0;"
        )
        moving = "live" if self._gait_live else "held"
        self._gait_intent_label.setText(
            f"intent: vx={vx:+.2f} vy={vy:+.2f} turn={rotation:+.2f} speed={speed:.0f} ({moving})"
        )
        a = self._gait_angles
        self._gait_angles_label.setText(
            f"commanded: coxa {deg_from_neutral('coxa', a.coxa_deg):+.1f}  "
            f"femur {deg_from_neutral('femur', a.femur_deg):+.1f}  "
            f"tibia {deg_from_neutral('tibia', a.tibia_deg):+.1f} (deg from neutral)"
        )
        self._view.set_gait_overlay(
            gait.trajectory(self._leg_index, vx, vy, speed, rotation, body_height),
            self._gait_phase,
            True,
            self._gait_zoom_check.isChecked(),
        )

    def _set_controls_enabled(self, enabled: bool) -> None:
        self._enabled_base = enabled
        self._release_button.setEnabled(enabled)
        self._drive_ik_button.setEnabled(enabled)
        # Channel wiring (board/channel combos) is deliberately left
        # alone here -- always enabled, at Qt's own default, since
        # setting up wiring is meaningful to do while disarmed too.
        # Step buttons are handled by _update_step_button_states(), which
        # also factors in the marked-limit rule -- not a blanket enable.

    def _send(self, command) -> Telemetry | None:
        return self.link.send_and_wait(command, timeout_s=_SEND_TIMEOUT_S)


def _limits_text(profile) -> str:
    if profile.min_deg_from_neutral is None and profile.max_deg_from_neutral is None:
        return "not bench-tested"
    lo = f"{profile.min_deg_from_neutral:.1f}" if profile.min_deg_from_neutral is not None else "?"
    hi = f"{profile.max_deg_from_neutral:.1f}" if profile.max_deg_from_neutral is not None else "?"
    return f"{lo} to {hi} deg from neutral"
