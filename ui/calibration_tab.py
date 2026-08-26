"""Calibration tab: arm/disarm gate, per-servo trim, full-table read, and
YAML export/import -- built against MockRobotLink's arm/disarm gate and
profile table (transport/mock_link.py), so it's fully usable with zero
hardware.

Unlike control_panel.py, this widget calls RobotLink directly rather than
going through MainWindow. Calibration writes are one-shot, self-contained
actions (arm, adjust one servo, read the table, export, import) that never
interact with the manual/AUTO_TRACK intent state machine MainWindow
coordinates -- routing them through MainWindow as pass-through signal
plumbing would add a layer with no real benefit. Manual motion commands
still go exclusively through MainWindow, since that IS a shared state
machine (current intent, mode) multiple things need to agree on.

Per docs/protocol.md Section 6, the actual safety mechanism here isn't the
arm/disarm gate (that guards against a stray/malformed packet, not
operator error) -- it's making a bad write cheap to undo via bulk
read/export/import. The arm/disarm UI matters, but the export reminder is
the part that's "hard to skip".

This tab only ever writes offset_us/sign (via calibrate/write_offsets).
min_pulse_us/max_pulse_us/note are written exclusively by the Bench Test
tab (its own arm gate, its own commands) -- but since both live in the
same ServoProfile record (docs/protocol.md Section 1), reading/exporting
here shows the full picture, including whatever bench testing has already
found. This tab's table is a snapshot as of the last "Read offsets" click,
not live-synced with the Bench tab -- re-read to see its updates here.
"""

import dataclasses
import time

import yaml
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from transport.generated_constants import CALIBRATION_ARM_TIMEOUT_S
from transport.link import RobotLink
from transport.protocol import (
    CALIBRATION_OFFSET_LIMIT_US,
    CalibrateCommand,
    CalibrationModeCommand,
    ProtocolError,
    ReadOffsetsCommand,
    SERVO_COUNT,
    ServoProfile,
    Telemetry,
    WriteOffsetsCommand,
)
from ui.servo_names import SERVO_NAMES

_SEND_TIMEOUT_S = 0.5  # bounded so a slow/real link can't freeze the UI for long
_UNEXPORTED_STYLE = "background-color: #e65100; color: white; padding: 6px; font-weight: bold;"


def _limits_text(profile: ServoProfile) -> str:
    if profile.min_pulse_us is None and profile.max_pulse_us is None:
        return "not bench-tested"
    lo = profile.min_pulse_us if profile.min_pulse_us is not None else "?"
    hi = profile.max_pulse_us if profile.max_pulse_us is not None else "?"
    return f"{lo}-{hi}us"


class CalibrationTab(QWidget):
    log_message = Signal(str)

    def __init__(self, link: RobotLink, parent=None) -> None:
        super().__init__(parent)
        self.link = link

        self._armed_since: float | None = None
        self._explicit_disarm_pending = False
        self._was_armed = False
        self._known_profiles: dict[int, ServoProfile] = {}
        self._exported_profiles: dict[int, ServoProfile] | None = None

        layout = QVBoxLayout(self)
        layout.addWidget(self._build_arm_section())
        layout.addWidget(self._build_servo_section())
        layout.addWidget(self._build_table_section())
        layout.addWidget(self._build_export_section())
        layout.addStretch(1)

        self._refresh_servo_controls_from_known()

    # --- construction -----------------------------------------------------

    def _build_arm_section(self) -> QGroupBox:
        box = QGroupBox("Arm / disarm")
        layout = QHBoxLayout(box)

        self._armed_label = QLabel("DISARMED")
        self._armed_label.setFocusPolicy(Qt.NoFocus)
        self._countdown_label = QLabel("")
        self._countdown_label.setFocusPolicy(Qt.NoFocus)

        self._arm_button = QPushButton("Arm")
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

    def _build_servo_section(self) -> QGroupBox:
        box = QGroupBox("Per-servo trim")
        layout = QVBoxLayout(box)

        selector_row = QHBoxLayout()
        self._servo_combo = QComboBox()
        self._servo_combo.setFocusPolicy(Qt.NoFocus)
        for i, name in enumerate(SERVO_NAMES):
            self._servo_combo.addItem(f"{i}: {name}")
        self._servo_combo.currentIndexChanged.connect(self._on_servo_selected)
        selector_row.addWidget(QLabel("servo"))
        selector_row.addWidget(self._servo_combo)
        layout.addLayout(selector_row)

        offset_row = QHBoxLayout()
        self._offset_slider = QSlider(Qt.Horizontal)
        self._offset_slider.setRange(-CALIBRATION_OFFSET_LIMIT_US, CALIBRATION_OFFSET_LIMIT_US)
        self._offset_slider.setFocusPolicy(Qt.NoFocus)
        self._offset_value_label = QLabel("0")
        self._offset_value_label.setFixedWidth(48)
        self._offset_slider.valueChanged.connect(
            lambda v: self._offset_value_label.setText(str(v))
        )
        offset_row.addWidget(QLabel("offset (us)"))
        offset_row.addWidget(self._offset_slider)
        offset_row.addWidget(self._offset_value_label)
        layout.addLayout(offset_row)

        sign_row = QHBoxLayout()
        self._sign_combo = QComboBox()
        self._sign_combo.setFocusPolicy(Qt.NoFocus)
        self._sign_combo.addItems(["+1", "-1"])
        sign_row.addWidget(QLabel("sign"))
        sign_row.addWidget(self._sign_combo)

        self._apply_button = QPushButton("Apply")
        self._apply_button.setFocusPolicy(Qt.NoFocus)
        self._apply_button.clicked.connect(self._on_apply_clicked)
        sign_row.addWidget(self._apply_button)
        layout.addLayout(sign_row)

        return box

    def _build_table_section(self) -> QGroupBox:
        box = QGroupBox("Servo profile table")
        layout = QVBoxLayout(box)

        self._read_button = QPushButton("Read profiles from robot")
        self._read_button.setFocusPolicy(Qt.NoFocus)
        self._read_button.clicked.connect(self._on_read_clicked)
        layout.addWidget(self._read_button)

        self._table = QTableWidget(SERVO_COUNT, 6)
        self._table.setHorizontalHeaderLabels(
            ["#", "servo", "offset (us)", "sign", "bench limits", "note"]
        )
        self._table.setFocusPolicy(Qt.NoFocus)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        for i, name in enumerate(SERVO_NAMES):
            self._table.setItem(i, 0, QTableWidgetItem(str(i)))
            self._table.setItem(i, 1, QTableWidgetItem(name))
            self._table.setItem(i, 2, QTableWidgetItem("—"))
            self._table.setItem(i, 3, QTableWidgetItem("—"))
            self._table.setItem(i, 4, QTableWidgetItem("—"))
            self._table.setItem(i, 5, QTableWidgetItem(""))
        layout.addWidget(self._table)
        return box

    def _build_export_section(self) -> QWidget:
        row = QHBoxLayout()
        export_button = QPushButton("Export to YAML")
        export_button.setFocusPolicy(Qt.NoFocus)
        export_button.clicked.connect(self._on_export_clicked)
        import_button = QPushButton("Import from YAML")
        import_button.setFocusPolicy(Qt.NoFocus)
        import_button.clicked.connect(self._on_import_clicked)
        row.addWidget(export_button)
        row.addWidget(import_button)

        container = QWidget()
        outer = QVBoxLayout(container)
        outer.addLayout(row)

        self._unexported_banner = QLabel()
        self._unexported_banner.setWordWrap(True)
        self._unexported_banner.hide()
        outer.addWidget(self._unexported_banner)
        return container

    # --- periodic refresh, called from MainWindow's central tick ----------

    def tick(self, telemetry: Telemetry | None) -> None:
        """The countdown is a client-side estimate: CALIBRATION_ARM_TIMEOUT_S
        (from transport/generated_constants.py) plus the last time *this
        process* sent an arm/refresh, re-synced against the wire
        `calibration_armed` boolean every tick so it can never show
        "counting down" once the robot has actually disarmed. It assumes
        the link's actual arm timeout matches that constant, which is true
        for how app.py constructs a link (no override) but would be wrong
        against a RobotLink deliberately built with a different
        calibration_arm_timeout_s -- e.g. for fast manual testing. Not
        wire-verified, same tradeoff as constants_warning's link_timeout_s
        check makes explicit for the failsafe timeout (docs/protocol.md
        Section 5), just without a telemetry field backing it here."""
        armed = telemetry.calibration_armed if telemetry is not None else False

        if armed and self._armed_since is not None:
            remaining = max(0.0, CALIBRATION_ARM_TIMEOUT_S - (time.monotonic() - self._armed_since))
            self._countdown_label.setText(f"auto-disarms in {remaining:0.0f}s")
        else:
            self._countdown_label.setText("")

        if armed:
            self._armed_label.setText("ARMED")
            self._armed_label.setStyleSheet("color: #ffb300; font-weight: bold;")
        else:
            self._armed_label.setText("DISARMED")
            self._armed_label.setStyleSheet("")
            self._armed_since = None

        self._arm_button.setEnabled(not armed)
        self._disarm_button.setEnabled(armed)

        if self._was_armed and not armed:
            if not self._explicit_disarm_pending and self._offsets_changed_since_export():
                self.log_message.emit(
                    "WARNING: calibration auto-disarmed with unexported offset changes"
                )
                self._show_unexported_banner()
            self._explicit_disarm_pending = False
        self._was_armed = armed

    # --- arm/disarm ---------------------------------------------------

    def _on_arm_clicked(self) -> None:
        telemetry = self._send(CalibrationModeCommand(armed=True))
        if telemetry is not None and telemetry.ok:
            self._armed_since = time.monotonic()
            self.log_message.emit("calibration armed")
        else:
            self.log_message.emit("failed to arm calibration")

    def _on_disarm_clicked(self) -> None:
        if self._offsets_changed_since_export():
            choice = QMessageBox.question(
                self,
                "Unexported changes",
                "Offsets have changed since your last export. Export now before disarming?",
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
            )
            if choice == QMessageBox.Cancel:
                return
            if choice == QMessageBox.Yes and not self._export_offsets():
                return  # export failed or was cancelled -- don't disarm

        self._explicit_disarm_pending = True
        telemetry = self._send(CalibrationModeCommand(armed=False))
        if telemetry is not None and telemetry.ok:
            self.log_message.emit("calibration disarmed")
            self._hide_unexported_banner()
        else:
            self.log_message.emit("failed to disarm calibration")

    # --- per-servo trim -----------------------------------------------

    def _on_servo_selected(self, _index: int) -> None:
        self._refresh_servo_controls_from_known()

    def _refresh_servo_controls_from_known(self) -> None:
        servo_index = self._servo_combo.currentIndex()
        profile = self._known_profiles.get(servo_index, ServoProfile(servo_index))
        self._offset_slider.setValue(profile.offset_us)
        self._sign_combo.setCurrentIndex(0 if profile.sign == 1 else 1)

    def _on_apply_clicked(self) -> None:
        servo_index = self._servo_combo.currentIndex()
        offset_us = self._offset_slider.value()
        sign = 1 if self._sign_combo.currentText() == "+1" else -1

        try:
            command = CalibrateCommand(servo_index=servo_index, offset_us=offset_us, sign=sign)
        except ProtocolError as exc:
            self.log_message.emit(f"rejected: {exc}")
            return

        telemetry = self._send(command)
        if telemetry is None:
            self.log_message.emit(f"no response applying servo {servo_index}")
            return
        if not telemetry.ok:
            self.log_message.emit(f"rejected by robot: {telemetry.error}")
            return

        # Merge onto whatever's locally known -- never blow away a cached
        # bench limit/note just because this tab only knows about offset/sign.
        existing = self._known_profiles.get(servo_index, ServoProfile(servo_index))
        updated = dataclasses.replace(existing, offset_us=offset_us, sign=sign)
        self._known_profiles[servo_index] = updated
        self._armed_since = time.monotonic()  # matches the robot's own refresh-on-write
        self._update_table_row(updated)
        self.log_message.emit(f"applied servo {servo_index} ({SERVO_NAMES[servo_index]}): {offset_us}us x{sign}")

    # --- bulk read ------------------------------------------------------

    def _on_read_clicked(self) -> None:
        telemetry = self._send(ReadOffsetsCommand())
        if telemetry is None or not telemetry.ok or telemetry.profiles is None:
            self.log_message.emit("failed to read profile table")
            return
        for profile in telemetry.profiles:
            self._known_profiles[profile.servo_index] = profile
            self._update_table_row(profile)
        self._refresh_servo_controls_from_known()
        self.log_message.emit(f"read {len(telemetry.profiles)} servo profiles from robot")

    def _update_table_row(self, profile: ServoProfile) -> None:
        i = profile.servo_index
        self._table.setItem(i, 2, QTableWidgetItem(str(profile.offset_us)))
        self._table.setItem(i, 3, QTableWidgetItem(f"{profile.sign:+d}"))
        self._table.setItem(i, 4, QTableWidgetItem(_limits_text(profile)))
        self._table.setItem(i, 5, QTableWidgetItem(profile.note))

    # --- export / import --------------------------------------------------

    def _on_export_clicked(self) -> None:
        self._export_offsets()

    def _export_offsets(self) -> bool:
        if not self._known_profiles:
            QMessageBox.warning(
                self, "Nothing to export",
                "No known servo profiles yet -- read the table from the robot first.",
            )
            return False

        path, _filter = QFileDialog.getSaveFileName(
            self, "Export servo profiles", "servo_profiles.yaml", "YAML files (*.yaml *.yml)"
        )
        if not path:
            return False

        entries = [
            self._known_profiles[i].to_dict()
            for i in sorted(self._known_profiles)
        ]
        with open(path, "w") as f:
            yaml.safe_dump({"offsets": entries}, f, sort_keys=False)

        self._exported_profiles = dict(self._known_profiles)
        self._hide_unexported_banner()
        self.log_message.emit(f"exported {len(entries)} servo profiles to {path}")
        return True

    def _on_import_clicked(self) -> None:
        path, _filter = QFileDialog.getOpenFileName(
            self, "Import servo profiles", "", "YAML files (*.yaml *.yml)"
        )
        if not path:
            return

        try:
            with open(path) as f:
                raw = yaml.safe_load(f) or {}
            entries = raw.get("offsets", [])
            profiles = [ServoProfile.from_dict(entry) for entry in entries]
        except (OSError, yaml.YAMLError, ProtocolError) as exc:
            QMessageBox.critical(self, "Import failed", f"Could not load {path}:\n{exc}")
            return

        if not profiles:
            QMessageBox.warning(self, "Import failed", f"{path} contains no servo profiles.")
            return

        choice = QMessageBox.question(
            self,
            "Confirm bulk write",
            f"Write {len(profiles)} trim corrections (offset/sign only -- bench limits and "
            f"notes are not restored by this import, see the Bench Test tab) from {path} "
            "to the robot now?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if choice != QMessageBox.Yes:
            return

        try:
            command = WriteOffsetsCommand(offsets=profiles)
        except ProtocolError as exc:
            QMessageBox.critical(self, "Import failed", f"Invalid offset table:\n{exc}")
            return

        telemetry = self._send(command)
        if telemetry is None:
            self.log_message.emit("no response writing imported offsets")
            return
        if not telemetry.ok:
            QMessageBox.critical(
                self, "Write rejected",
                f"Robot rejected the bulk write: {telemetry.error}\n"
                "(calibration must be armed first)",
            )
            return

        # Only offset_us/sign actually changed on the robot -- merge just
        # those fields locally too, don't adopt limits/note from the file
        # (which may be stale relative to what the robot actually has).
        for imported in profiles:
            existing = self._known_profiles.get(imported.servo_index, ServoProfile(imported.servo_index))
            updated = dataclasses.replace(
                existing, offset_us=imported.offset_us, sign=imported.sign
            )
            self._known_profiles[imported.servo_index] = updated
            self._update_table_row(updated)
        self._armed_since = time.monotonic()
        self._refresh_servo_controls_from_known()
        self.log_message.emit(f"wrote {len(profiles)} imported offset corrections to robot")

    # --- helpers ------------------------------------------------------

    def _offsets_changed_since_export(self) -> bool:
        if not self._known_profiles:
            return False
        return self._known_profiles != self._exported_profiles

    def _show_unexported_banner(self) -> None:
        self._unexported_banner.setText(
            "⚠ Offsets changed since your last export -- export recommended."
        )
        self._unexported_banner.setStyleSheet(_UNEXPORTED_STYLE)
        self._unexported_banner.show()

    def _hide_unexported_banner(self) -> None:
        self._unexported_banner.hide()

    def _send(self, command) -> Telemetry | None:
        return self.link.send_and_wait(command, timeout_s=_SEND_TIMEOUT_S)
