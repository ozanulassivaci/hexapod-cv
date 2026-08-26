"""SimView: top-down 2D visualization of a SimRobotLink's current state --
body position/heading and which legs are in stance (foot planted) vs
swing (foot lifted). Physical fidelity is explicitly not the goal (see
simulator/robot_state.py's docstring) -- this exists to see that the
right commands produce the right intent, and to catch gait bugs before
they reach real servos, not to look like a real robot.

No internal QTimer -- tick() is called from MainWindow's existing central
tick (the same "one timer drives every tab" pattern
CalibrationTab.tick()/BenchTab.tick() already use), it just triggers a
repaint; paintEvent() pulls a fresh snapshot itself.
"""

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from simulator.sim_link import SimRobotLink

_BACKGROUND_COLOR = QColor(25, 25, 25)
_BODY_COLOR = QColor(70, 130, 200)
_DISCONNECTED_COLOR = QColor(120, 120, 120)
_STANCE_COLOR = QColor(60, 180, 75)
_SWING_COLOR = QColor(235, 160, 30)
_LEG_LINE_COLOR = QColor(90, 90, 90)
_TEXT_COLOR = QColor(230, 230, 230)

_BODY_RADIUS_PX = 22.0
_FOOT_RADIUS_PX = 7.0
_HEADING_ARROW_MM = 120.0
_WORLD_SPAN_MM = 900.0  # roughly how much world (mm) fits across the shorter widget dimension


class SimView(QWidget):
    def __init__(self, link: SimRobotLink, parent=None) -> None:
        super().__init__(parent)
        self.link = link
        self.setMinimumSize(360, 360)

    def tick(self) -> None:
        self.update()  # just schedules a repaint -- paintEvent reads fresh state itself

    def _pixels_per_mm(self) -> float:
        return max(1.0, min(self.width(), self.height())) / _WORLD_SPAN_MM

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override naming)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), _BACKGROUND_COLOR)

        snap = self.link.snapshot()
        scale = self._pixels_per_mm()
        origin = QPointF(self.width() / 2.0, self.height() / 2.0)

        def to_screen(x_mm: float, y_mm: float) -> QPointF:
            # World frame: +x forward (screen up), +y right (screen right).
            return QPointF(origin.x() + y_mm * scale, origin.y() - x_mm * scale)

        heading_rad = math.radians(snap.heading_deg)
        cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)

        body_pt = to_screen(snap.body_x_mm, snap.body_y_mm)

        # Foot positions are body-relative (leg-local FK output, see
        # robot/kinematics.py) -- rotate by heading, then translate by
        # body position, to get world-frame points.
        foot_points = []
        for foot in snap.foot_positions:
            world_x = snap.body_x_mm + (foot.x * cos_h - foot.y * sin_h)
            world_y = snap.body_y_mm + (foot.x * sin_h + foot.y * cos_h)
            foot_points.append(to_screen(world_x, world_y))

        tip_world_x = snap.body_x_mm + _HEADING_ARROW_MM * cos_h
        tip_world_y = snap.body_y_mm + _HEADING_ARROW_MM * sin_h
        tip_pt = to_screen(tip_world_x, tip_world_y)

        painter.setPen(QPen(_LEG_LINE_COLOR, 1))
        for pt in foot_points:
            painter.drawLine(body_pt, pt)

        body_color = _BODY_COLOR if snap.connected else _DISCONNECTED_COLOR
        painter.setBrush(QBrush(body_color))
        painter.setPen(QPen(Qt.white, 1))
        painter.drawEllipse(body_pt, _BODY_RADIUS_PX, _BODY_RADIUS_PX)

        painter.setPen(QPen(Qt.white, 3))
        painter.drawLine(body_pt, tip_pt)

        for pt, stance in zip(foot_points, snap.stance):
            painter.setBrush(QBrush(_STANCE_COLOR if stance else _SWING_COLOR))
            painter.setPen(QPen(Qt.white, 1))
            painter.drawEllipse(pt, _FOOT_RADIUS_PX, _FOOT_RADIUS_PX)

        painter.setPen(QPen(_TEXT_COLOR))
        status = "connected" if snap.connected else "DISCONNECTED (failsafe -- gait frozen)"
        painter.drawText(QRectF(8, 4, self.width() - 16, 20), Qt.AlignLeft, f"phase={snap.gait_phase:.2f}  {status}")
