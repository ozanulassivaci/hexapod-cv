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
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from control.single_leg import EXPLORATION_ORDER, TEST_LEG, deg_from_neutral, is_step_allowed
from robot.gait import (
    GAIT_ENVELOPE_COXA_MAX_DEG,
    GAIT_ENVELOPE_COXA_MIN_DEG,
    GAIT_ENVELOPE_FEMUR_MAX_DEG,
    GAIT_ENVELOPE_FEMUR_MIN_DEG,
    GAIT_ENVELOPE_TIBIA_MAX_DEG,
    GAIT_ENVELOPE_TIBIA_MIN_DEG,
)
from robot.kinematics import (
    LEGS,
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

        self._set_controls_enabled(False)
        self._refresh_known_limits()

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
        row.addWidget(self._joint_mode_button)
        row.addWidget(self._ik_mode_button)
        return container

    def _set_mode(self, index: int) -> None:
        self._stack.setCurrentIndex(index)
        self._joint_mode_button.setChecked(index == 0)
        self._ik_mode_button.setChecked(index == 1)

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
        self._leg_index = index
        self._refresh_known_limits()

    def _servo_index(self, joint: str) -> int:
        return self._leg_index * 3 + _JOINT_OFFSET[joint]

    def _refresh_known_limits(self) -> None:
        telemetry = self._send(ReadOffsetsCommand())
        if telemetry is None or not telemetry.ok or telemetry.profiles is None:
            return
        for joint in ("coxa", "femur", "tibia"):
            profile = telemetry.profiles[self._servo_index(joint)]
            self._known_limits[joint] = (profile.min_deg_from_neutral, profile.max_deg_from_neutral)
            self._limits_label[joint].setText(f"known limits: {_limits_text(profile)}")
        self._view.tick()

    # --- release -----------------------------------------------------

    def _on_release_clicked(self) -> None:
        self._release_all()
        self.log_message.emit("test leg released (all three joints limp)")

    def _release_all(self) -> None:
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
        self.link.send(
            BenchPulseCommand(board=board, channel=channel, pulse_us=pulse_us, servo_index=servo_index)
        )
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
