"""Left-hand video panel: latest camera frame with detection boxes and an
FPS/frame-age overlay drawn over it, or a clear placeholder when there's no
camera. A missing camera is a normal, expected state here, not an error --
this widget is designed to spend most of its life with frame=None while
developing against MockRobotLink with no camera attached at all.

Purely a rendering widget: it holds no timing state of its own. FPS and
frame age are computed by whoever drives the tick (ui/main_window.py) and
passed in, so this widget's only job is "given a frame and some numbers,
draw them."
"""

import cv2
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget

from perception.detection import Detection

_BOX_COLOR_BGR = (255, 0, 0)
_OVERLAY_COLOR_BGR = (0, 255, 0)
_PLACEHOLDER_COLOR_BGR = (0, 0, 255)
# Floor only, not a cap -- small enough that the placeholder text and a
# detection box are still legible, big enough to not look broken. The
# label itself has no maximum: it grows to fill whatever space the
# Operate tab's layout gives it (see resizeEvent below).
_MIN_LABEL_SIZE = (320, 240)


class VideoPanel(QWidget):
    def __init__(self, width: int, height: int, parent=None) -> None:
        super().__init__(parent)
        # Native camera resolution (e.g. DroidCam's 640x480) -- used to
        # build the placeholder frame at the right aspect ratio, not as a
        # display size. Display size follows the label's actual allocated
        # size, recomputed on every resize (_render below).
        self._width = width
        self._height = height
        self._last_frame: np.ndarray | None = None
        self._last_detections: list[Detection] = []
        self._last_fps = 0.0
        self._last_frame_age_ms: float | None = None

        self._label = QLabel()
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setStyleSheet("background-color: black; color: white;")
        self._label.setMinimumSize(*_MIN_LABEL_SIZE)
        self._label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._label)

        self.update_frame(frame=None, detections=[], fps=0.0, frame_age_ms=None)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        # The label's pixmap doesn't auto-rescale on resize (QLabel only
        # does that with setScaledContents, which ignores aspect ratio) --
        # re-render the last frame at the new size instead.
        self._render()

    def update_frame(
        self,
        frame: np.ndarray | None,
        detections: list[Detection],
        fps: float,
        frame_age_ms: float | None,
    ) -> None:
        self._last_frame = frame
        self._last_detections = detections
        self._last_fps = fps
        self._last_frame_age_ms = frame_age_ms
        self._render()

    def _render(self) -> None:
        frame = self._last_frame
        if frame is None:
            pixmap = self._placeholder_pixmap("waiting for camera...")
        else:
            # stream.read() hands back its internal buffer by reference, not
            # a copy -- draw on a copy so this widget never defaces what a
            # future reader of the stream would consider "the latest frame".
            frame = frame.copy()
            _draw_detections(frame, self._last_detections)
            _draw_overlay(frame, self._last_fps, self._last_frame_age_ms)
            pixmap = _frame_to_pixmap(frame)

        target = self._label.size()
        if target.width() <= 0 or target.height() <= 0:
            return  # not laid out yet -- resizeEvent fires again once it is
        scaled = pixmap.scaled(target, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._label.setPixmap(scaled)

    def _placeholder_pixmap(self, message: str) -> QPixmap:
        frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        cv2.putText(
            frame, message, (16, self._height // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, _PLACEHOLDER_COLOR_BGR, 2, cv2.LINE_AA,
        )
        return _frame_to_pixmap(frame)


def _draw_detections(frame: np.ndarray, detections: list[Detection]) -> None:
    for det in detections:
        x, y, w, h = det.bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), _BOX_COLOR_BGR, 2)
        label = f"{det.label} ({det.cx:+.2f}, {det.cy:+.2f})"
        cv2.putText(
            frame, label, (x, max(y - 8, 0)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, _BOX_COLOR_BGR, 1, cv2.LINE_AA,
        )


def _draw_overlay(frame: np.ndarray, fps: float, frame_age_ms: float | None) -> None:
    age_text = f"{frame_age_ms:.0f}ms" if frame_age_ms is not None else "n/a"
    cv2.putText(
        frame, f"FPS: {fps:.1f}  age: {age_text}", (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, _OVERLAY_COLOR_BGR, 1, cv2.LINE_AA,
    )


def _frame_to_pixmap(frame_bgr: np.ndarray) -> QPixmap:
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    height, width = rgb.shape[:2]
    image = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888)
    # QImage doesn't own rgb's buffer -- copy before rgb (a local var) goes
    # out of scope and its memory becomes eligible for reuse/GC.
    return QPixmap.fromImage(image.copy())
