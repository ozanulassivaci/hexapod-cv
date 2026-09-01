"""2D side + top view visualization for the Test Leg tab (bring-up plan's
Part 6). Two flat views, not one 3D view -- two clear 2D views beat one
confusing 3D one, and each maps directly onto a quantity this project
already reasons about elsewhere:

- **Side view** is the femur/tibia plane: (l_forward, z), the exact pair
  forward_kinematics() computes before ever applying the coxa rotation
  (robot/kinematics.py). Deliberately independent of coxa_deg -- coxa
  only changes which *world* direction this plane faces, never the
  plane's own internal geometry, so the side view never needs to know
  the current coxa angle at all.
- **Top view** is coxa rotation: a single line from the coxa's own
  origin, at the current coxa angle, whose length is the leg's current
  horizontal reach (l_forward) -- "which way is the leg currently
  pointing, seen from above."

Reads current_deg/known_limits by reference (the exact dict objects
ui/single_leg_tab.py mutates in place), so paintEvent always sees the
latest values on the next repaint with no explicit push -- the same
"pull fresh state on repaint" pattern simulator/sim_view.py already
establishes for its own canvas.

Every arc/wedge below is built by sampling points along the curve and
connecting them with QPainterPath.lineTo(), not Qt's native
drawArc()/drawPie() angle API -- deliberately, to avoid needing to
verify Qt's own angle-sign convention under a coordinate system with a
flipped (screen-down-positive) Y axis. Every point this module ever
plots goes through the two to_screen_*() functions below, so there is
exactly one place the math could be wrong, not one per shape.
"""

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget

from control.single_leg import safety_zone_color
from robot.gait import (
    GAIT_ENVELOPE_COXA_MAX_DEG,
    GAIT_ENVELOPE_COXA_MIN_DEG,
    GAIT_ENVELOPE_FEMUR_MAX_DEG,
    GAIT_ENVELOPE_FEMUR_MIN_DEG,
    GAIT_ENVELOPE_TIBIA_MAX_DEG,
    GAIT_ENVELOPE_TIBIA_MIN_DEG,
)
from robot.kinematics import (
    COXA_MAX_DEG,
    COXA_MIN_DEG,
    FEMUR_LENGTH_MM,
    FEMUR_MAX_DEG,
    FEMUR_MIN_DEG,
    SAFE_LIMIT_MARGIN_DEG,
    TIBIA_LENGTH_MM,
    TIBIA_MAX_DEG,
    TIBIA_MIN_DEG,
)

_BACKGROUND_COLOR = QColor(25, 25, 25)
_TEXT_COLOR = QColor(230, 230, 230)
_WORKSPACE_FILL = QColor(60, 60, 70, 90)
_ENVELOPE_FILL = QColor(70, 130, 200, 90)
_FORBIDDEN_FILL = QColor(180, 40, 40, 110)
_JOINT_COLOR = QColor(230, 230, 230)
_ZONE_COLOR = {
    "green": QColor(60, 180, 75),
    "amber": QColor(235, 160, 30),
    "red": QColor(220, 50, 50),
    "unknown": QColor(150, 150, 150),
}

_ARC_SAMPLES = 32
_REACH_MM = FEMUR_LENGTH_MM + TIBIA_LENGTH_MM


def _femur_tibia_points(femur_deg: float, tibia_deg: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """(elbow, foot) in (l_forward, z) mm -- mirrors
    forward_kinematics()'s own beta/gamma math exactly, minus the coxa
    rotation and origin translation this view never applies."""
    beta = math.radians(femur_deg)
    gamma = math.radians(tibia_deg)
    elbow = (FEMUR_LENGTH_MM * math.cos(beta), FEMUR_LENGTH_MM * math.sin(beta))
    foot = (
        elbow[0] + TIBIA_LENGTH_MM * math.cos(beta + gamma),
        elbow[1] + TIBIA_LENGTH_MM * math.sin(beta + gamma),
    )
    return elbow, foot


def _convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Andrew's monotone chain, no external dependency -- points ordered
    counter-clockwise, first point not repeated at the end. Used once
    (see _WORKSPACE_HULL_MM below), not per repaint, so this doesn't need
    to be fast, just correct and dependency-free."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _compute_workspace_hull_mm(femur_samples: int = 60, tibia_samples: int = 60) -> list[tuple[float, float]]:
    """Convex hull of a dense (femur, tibia) grid's FK output, in
    (l_forward, z) mm -- an approximation of the reachable workspace's
    outer boundary, not an exact reachability computation (a 2-link
    arm's true reachable set generally isn't convex, so this is a
    conservative over-approximation: it never shows less than what's
    actually reachable, but may show a bit more in places). Computed
    once at import time (the joint-limit constants it depends on are
    fixed), not per repaint -- see _WORKSPACE_HULL_MM."""
    points = []
    for i in range(femur_samples + 1):
        femur_deg = FEMUR_MIN_DEG + (FEMUR_MAX_DEG - FEMUR_MIN_DEG) * i / femur_samples
        for j in range(tibia_samples + 1):
            tibia_deg = TIBIA_MIN_DEG + (TIBIA_MAX_DEG - TIBIA_MIN_DEG) * j / tibia_samples
            _, foot = _femur_tibia_points(femur_deg, tibia_deg)
            points.append(foot)
    return _convex_hull(points)


_WORKSPACE_HULL_MM = _compute_workspace_hull_mm()


class SingleLegView(QWidget):
    """current_deg/known_limits: the exact dict objects owned by
    ui/single_leg_tab.py's SingleLegTab, passed by reference -- this
    widget never copies or caches them, only reads fresh on each
    repaint. tick() (called from the tab, itself called from
    MainWindow's central tick) just schedules that repaint."""

    def __init__(self, current_deg: dict, known_limits: dict, parent=None) -> None:
        super().__init__(parent)
        self._current_deg = current_deg
        self._known_limits = known_limits

        layout = QHBoxLayout(self)
        self._side_canvas = _SideViewCanvas(current_deg, known_limits)
        self._top_canvas = _TopViewCanvas(current_deg, known_limits)
        layout.addWidget(self._side_canvas, 1)
        layout.addWidget(self._top_canvas, 1)

    def tick(self) -> None:
        self._side_canvas.update()
        self._top_canvas.update()


class _BaseCanvas(QWidget):
    def __init__(self, current_deg: dict, known_limits: dict, parent=None) -> None:
        super().__init__(parent)
        self._current_deg = current_deg
        self._known_limits = known_limits
        self.setMinimumSize(260, 260)

    def _origin_and_scale(self) -> tuple[QPointF, float]:
        margin_px = 24.0
        available = max(40.0, min(self.width(), self.height()) - 2 * margin_px)
        scale = available / (2.0 * _REACH_MM)
        return QPointF(self.width() / 2.0, self.height() / 2.0), scale


class _SideViewCanvas(_BaseCanvas):
    def to_screen(self, l_forward_mm: float, z_mm: float, origin: QPointF, scale: float) -> QPointF:
        # +l_forward = screen right, +z = up (screen up = smaller pixel y).
        return QPointF(origin.x() + l_forward_mm * scale, origin.y() - z_mm * scale)

    def _arc_path(
        self, center: tuple[float, float], radius_mm: float, lo_deg: float, hi_deg: float, origin, scale
    ) -> list[QPointF]:
        pts = []
        for i in range(_ARC_SAMPLES + 1):
            a = math.radians(lo_deg + (hi_deg - lo_deg) * i / _ARC_SAMPLES)
            x = center[0] + radius_mm * math.cos(a)
            z = center[1] + radius_mm * math.sin(a)
            pts.append(self.to_screen(x, z, origin, scale))
        return pts

    def _wedge_path(self, pivot_screen: QPointF, arc_pts: list[QPointF]) -> QPainterPath:
        path = QPainterPath(pivot_screen)
        for pt in arc_pts:
            path.lineTo(pt)
        path.closeSubpath()
        return path

    def _workspace_polygon(self, origin: QPointF, scale: float) -> QPainterPath:
        """The cached convex hull (_WORKSPACE_HULL_MM) transformed to
        screen space -- see _compute_workspace_hull_mm()'s own docstring
        for what this approximates and why. Not a safety boundary either
        way; Part 4's GatedServoDriver/LimitMode is what actually
        enforces anything."""
        points = [self.to_screen(x, z, origin, scale) for x, z in _WORKSPACE_HULL_MM]
        path = QPainterPath(points[0])
        for pt in points[1:]:
            path.lineTo(pt)
        path.closeSubpath()
        return path

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override naming)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), _BACKGROUND_COLOR)

        origin, scale = self._origin_and_scale()
        femur_deg = self._current_deg["femur"]
        tibia_deg = self._current_deg["tibia"]
        elbow, foot = _femur_tibia_points(femur_deg, tibia_deg)
        coxa_screen = self.to_screen(0.0, 0.0, origin, scale)
        elbow_screen = self.to_screen(elbow[0], elbow[1], origin, scale)
        foot_screen = self.to_screen(foot[0], foot[1], origin, scale)

        # Layered largest-to-smallest: reachable workspace (outer
        # context), gait envelope (what gait actually needs), forbidden
        # zones (marked mechanical limits, drawn last so they're never
        # hidden under the others), then the live stick figure on top.
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(_WORKSPACE_FILL))
        painter.drawPath(self._workspace_polygon(origin, scale))

        painter.setBrush(QBrush(_ENVELOPE_FILL))
        femur_env_arc = self._arc_path(
            (0.0, 0.0), FEMUR_LENGTH_MM, GAIT_ENVELOPE_FEMUR_MIN_DEG, GAIT_ENVELOPE_FEMUR_MAX_DEG, origin, scale
        )
        painter.drawPath(self._wedge_path(coxa_screen, femur_env_arc))
        tibia_env_arc = self._arc_path(
            elbow,
            TIBIA_LENGTH_MM,
            femur_deg + GAIT_ENVELOPE_TIBIA_MIN_DEG,
            femur_deg + GAIT_ENVELOPE_TIBIA_MAX_DEG,
            origin,
            scale,
        )
        painter.drawPath(self._wedge_path(elbow_screen, tibia_env_arc))

        painter.setBrush(QBrush(_FORBIDDEN_FILL))
        self._draw_forbidden_zones(painter, "femur", (0.0, 0.0), FEMUR_LENGTH_MM, 0.0, coxa_screen, origin, scale)
        self._draw_forbidden_zones(painter, "tibia", elbow, TIBIA_LENGTH_MM, femur_deg, elbow_screen, origin, scale)

        # Each segment gets its own joint's color, independently -- not
        # combined into one verdict for the whole leg. A combined
        # "worst of the two" verdict needs a severity ranking between
        # "red" and "unknown", and there isn't a non-arbitrary one: is a
        # joint confirmed close to its limit worse than a joint nobody's
        # assessed at all, or the other way around? Coloring each
        # segment on its own avoids inventing an answer to that, and
        # is strictly more informative -- nothing about the tibia's
        # state gets hidden by the femur's, or vice versa.
        femur_zone = safety_zone_color(
            femur_deg, self._known_limits["femur"][0], self._known_limits["femur"][1], SAFE_LIMIT_MARGIN_DEG
        )
        tibia_zone = safety_zone_color(
            tibia_deg, self._known_limits["tibia"][0], self._known_limits["tibia"][1], SAFE_LIMIT_MARGIN_DEG
        )
        painter.setPen(QPen(_ZONE_COLOR[femur_zone], 4))
        painter.drawLine(coxa_screen, elbow_screen)
        painter.setPen(QPen(_ZONE_COLOR[tibia_zone], 4))
        painter.drawLine(elbow_screen, foot_screen)

        painter.setPen(QPen(Qt.white, 1))
        painter.setBrush(QBrush(_JOINT_COLOR))
        for pt in (coxa_screen, elbow_screen, foot_screen):
            painter.drawEllipse(pt, 5.0, 5.0)

        painter.setPen(QPen(_TEXT_COLOR))
        painter.drawText(QRectF(6, 4, self.width() - 12, 20), Qt.AlignLeft, "side view (femur/tibia plane)")

    def _draw_forbidden_zones(self, painter, joint, center, radius_mm, angle_offset_deg, pivot_screen, origin, scale):
        marked_min, marked_max = self._known_limits[joint]
        joint_min = FEMUR_MIN_DEG if joint == "femur" else TIBIA_MIN_DEG
        joint_max = FEMUR_MAX_DEG if joint == "femur" else TIBIA_MAX_DEG
        if marked_min is not None and marked_min > joint_min:
            arc = self._arc_path(
                center, radius_mm, angle_offset_deg + joint_min, angle_offset_deg + marked_min, origin, scale
            )
            painter.drawPath(self._wedge_path(pivot_screen, arc))
        if marked_max is not None and marked_max < joint_max:
            arc = self._arc_path(
                center, radius_mm, angle_offset_deg + marked_max, angle_offset_deg + joint_max, origin, scale
            )
            painter.drawPath(self._wedge_path(pivot_screen, arc))


class _TopViewCanvas(_BaseCanvas):
    def to_screen(self, x_mm: float, y_mm: float, origin: QPointF, scale: float) -> QPointF:
        # +x = screen up (forward), +y = screen right -- matches
        # simulator/sim_view.py's own world-frame convention.
        return QPointF(origin.x() + y_mm * scale, origin.y() - x_mm * scale)

    def _arc_path(self, radius_mm: float, lo_deg: float, hi_deg: float, origin, scale) -> list[QPointF]:
        pts = []
        for i in range(_ARC_SAMPLES + 1):
            a = math.radians(lo_deg + (hi_deg - lo_deg) * i / _ARC_SAMPLES)
            pts.append(self.to_screen(radius_mm * math.cos(a), radius_mm * math.sin(a), origin, scale))
        return pts

    def _wedge_path(self, pivot_screen: QPointF, arc_pts: list[QPointF]) -> QPainterPath:
        path = QPainterPath(pivot_screen)
        for pt in arc_pts:
            path.lineTo(pt)
        path.closeSubpath()
        return path

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override naming)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), _BACKGROUND_COLOR)

        origin, scale = self._origin_and_scale()
        coxa_deg = self._current_deg["coxa"]
        femur_deg = self._current_deg["femur"]
        tibia_deg = self._current_deg["tibia"]
        _, foot = _femur_tibia_points(femur_deg, tibia_deg)
        l_forward = foot[0]  # current horizontal reach, whatever femur/tibia are doing right now

        origin_screen = self.to_screen(0.0, 0.0, origin, scale)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(_ENVELOPE_FILL))
        env_arc = self._arc_path(_REACH_MM, GAIT_ENVELOPE_COXA_MIN_DEG, GAIT_ENVELOPE_COXA_MAX_DEG, origin, scale)
        painter.drawPath(self._wedge_path(origin_screen, env_arc))

        painter.setBrush(QBrush(_FORBIDDEN_FILL))
        marked_min, marked_max = self._known_limits["coxa"]
        if marked_min is not None and marked_min > COXA_MIN_DEG:
            arc = self._arc_path(_REACH_MM, COXA_MIN_DEG, marked_min, origin, scale)
            painter.drawPath(self._wedge_path(origin_screen, arc))
        if marked_max is not None and marked_max < COXA_MAX_DEG:
            arc = self._arc_path(_REACH_MM, marked_max, COXA_MAX_DEG, origin, scale)
            painter.drawPath(self._wedge_path(origin_screen, arc))

        zone = safety_zone_color(coxa_deg, marked_min, marked_max, SAFE_LIMIT_MARGIN_DEG)
        tip_screen = self.to_screen(
            l_forward * math.cos(math.radians(coxa_deg)), l_forward * math.sin(math.radians(coxa_deg)), origin, scale
        )
        painter.setPen(QPen(_ZONE_COLOR[zone], 4))
        painter.drawLine(origin_screen, tip_screen)

        painter.setPen(QPen(Qt.white, 1))
        painter.setBrush(QBrush(_JOINT_COLOR))
        painter.drawEllipse(origin_screen, 5.0, 5.0)
        painter.drawEllipse(tip_screen, 5.0, 5.0)

        painter.setPen(QPen(_TEXT_COLOR))
        painter.drawText(QRectF(6, 4, self.width() - 12, 20), Qt.AlignLeft, "top view (coxa rotation)")
