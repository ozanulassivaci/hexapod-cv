"""SyntheticCameraStream: renders synthetic frames of one or more colored
targets whose apparent position depends on a SimRobotLink's live heading
-- the piece needed to close the vision -> tracker -> gait loop end to
end with no hardware anywhere in the chain (see the design discussion
this was built from). Frames are real BGR numpy arrays run through the
real HSVDetector, not synthesized Detection objects -- this exercises
the actual detection code, not a mock of it.

Drop-in compatible with stream.mjpeg_stream.MJPEGStream's interface
(start/stop/read/is_connected) so ui/main_window.py and app.py need zero
changes to accept either one.

Sign convention, explicitly chosen, not measured: increasing a target's
world bearing relative to the robot's heading increases cx (target
appears to move right). Chosen so the existing Tracker/TurnCommand.rate
convention (control/tracker.py's own documented assumption -- "to be
confirmed... once there's a robot to test it against") produces a
*converging* closed loop here. Whether this matches a real camera's
actual left/right mirroring is a hardware fact no amount of software can
determine, and this harness isn't attempting to -- it validates
control-loop dynamics (dead zone, gain, latency, lost-target behavior),
not camera geometry. See docs/protocol.md and control/tracker.py.

Latency is simulated explicitly and matters: the frame delivered *now*
reflects the robot's heading from `latency_s` ago, not the instantaneous
heading -- modeling the ~150-250ms real DroidCam/MJPEG pipeline this
project already measures via the frame-age overlay
(ui/video_panel.py)."""

import math
import random
import threading
import time

import cv2
import numpy as np

from simulator.sim_link import SimRobotLink

DEFAULT_FPS = 15.0  # roughly matches a real DroidCam MJPEG stream's practical rate
DEFAULT_LATENCY_S = 0.2  # midpoint of the ~150-250ms this project has measured against real DroidCam
DEFAULT_HALF_FOV_DEG = 30.0
DEFAULT_FRAME_SIZE = (640, 480)  # (width, height), matches config.py's StreamConfig-adjacent defaults

# HSV(120, 200, 150) -- the midpoint of config.py's default detection
# range (lower=(100,150,50), upper=(140,255,255)) -- converted to BGR so
# a rendered target reliably falls inside the real HSVDetector's default
# threshold without needing test-only detector configuration.
DEFAULT_TARGET_COLOR_BGR = (150, 32, 32)
BACKGROUND_COLOR_BGR = (40, 40, 40)  # dark gray, well outside the default blue HSV range


def _wrap_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


class SyntheticTarget:
    """One rendered object. world_bearing_deg is fixed -- a landmark, not
    something that moves on its own; only the robot's heading changes
    where it appears in frame. jitter_deg adds small independent
    per-frame bearing noise (simulating real contour-detection pixel
    jitter -- compression artifacts, lighting flicker -- not exact
    geometry, which a perfectly rendered circle otherwise has none of).
    flicker_probability drops the target from the frame entirely, that
    frame only, simulating intermittent detection dropout."""

    def __init__(
        self,
        world_bearing_deg: float,
        color_bgr: tuple[int, int, int] = DEFAULT_TARGET_COLOR_BGR,
        radius_px: int = 30,
        jitter_deg: float = 0.0,
        flicker_probability: float = 0.0,
    ) -> None:
        self.world_bearing_deg = world_bearing_deg
        self.color_bgr = color_bgr
        self.radius_px = radius_px
        self.jitter_deg = jitter_deg
        self.flicker_probability = flicker_probability


class SyntheticCameraStream:
    def __init__(
        self,
        link: SimRobotLink,
        targets: list[SyntheticTarget],
        fps: float = DEFAULT_FPS,
        latency_s: float = DEFAULT_LATENCY_S,
        half_fov_deg: float = DEFAULT_HALF_FOV_DEG,
        frame_size: tuple[int, int] = DEFAULT_FRAME_SIZE,
        rng_seed: int | None = None,
    ) -> None:
        self.link = link
        self.targets = targets
        self.fps = fps
        self.latency_s = latency_s
        self.half_fov_deg = half_fov_deg
        self.frame_size = frame_size
        self._rng = random.Random(rng_seed)

        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._timestamp: float | None = None
        self._connected = False

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> "SyntheticCameraStream":
        self._connected = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="SyntheticCameraStream")
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        self._connected = False

    def read(self) -> tuple[np.ndarray | None, float | None]:
        with self._lock:
            return self._frame, self._timestamp

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def _run(self) -> None:
        interval_s = 1.0 / self.fps
        while not self._stop_event.is_set():
            capture_time = time.monotonic()
            heading_deg = self.link.snapshot().heading_deg
            frame = self._render(heading_deg)

            # Simulate pipeline latency before this frame becomes
            # available -- timestamp stays the *capture* time, not
            # delivery time, so the frame-age overlay correctly reads
            # ~latency_s the instant it's delivered, matching what the
            # real MJPEG pipeline's age actually reflects.
            if self._stop_event.wait(self.latency_s):
                break
            with self._lock:
                self._frame = frame
                self._timestamp = capture_time

            elapsed = time.monotonic() - capture_time
            remaining = interval_s - elapsed
            if remaining > 0:
                self._stop_event.wait(remaining)

    def _render(self, heading_deg: float) -> np.ndarray:
        width, height = self.frame_size
        frame = np.full((height, width, 3), BACKGROUND_COLOR_BGR, dtype=np.uint8)
        for target in self.targets:
            if target.flicker_probability > 0 and self._rng.random() < target.flicker_probability:
                continue
            bearing = target.world_bearing_deg
            if target.jitter_deg > 0:
                bearing += self._rng.uniform(-target.jitter_deg, target.jitter_deg)
            relative_bearing = _wrap_deg(bearing - heading_deg)
            if abs(relative_bearing) > self.half_fov_deg:
                continue  # out of frame
            cx_norm = relative_bearing / self.half_fov_deg
            px = int(round(width / 2 + cx_norm * (width / 2)))
            py = height // 2
            cv2.circle(frame, (px, py), target.radius_px, target.color_bgr, -1)
        return frame

    def __enter__(self) -> "SyntheticCameraStream":
        return self.start()

    def __exit__(self, *exc_info) -> None:
        self.stop()
