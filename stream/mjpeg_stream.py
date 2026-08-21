import threading
import time

import cv2
import numpy as np


class MJPEGStream:
    """Reads an MJPEG-over-HTTP stream in a background thread and always
    hands back the latest frame. Frames are never queued: a slow consumer
    just sees a larger frame age, not a growing backlog of stale ones.

    Reconnects with exponential backoff if the source is unreachable or the
    connection drops mid-stream.
    """

    def __init__(
        self,
        url: str,
        retry_backoff_base_s: float = 1.0,
        retry_backoff_max_s: float = 30.0,
    ):
        self.url = url
        self.retry_backoff_base_s = retry_backoff_base_s
        self.retry_backoff_max_s = retry_backoff_max_s

        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._timestamp: float | None = None
        self._connected = False

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> "MJPEGStream":
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def read(self) -> tuple[np.ndarray | None, float | None]:
        """Return (latest_frame, timestamp) or (None, None) if nothing has
        arrived yet."""
        with self._lock:
            return self._frame, self._timestamp

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def _set_connected(self, connected: bool) -> None:
        with self._lock:
            self._connected = connected

    def _run(self) -> None:
        backoff = self.retry_backoff_base_s
        while not self._stop_event.is_set():
            cap = cv2.VideoCapture(self.url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not cap.isOpened():
                cap.release()
                self._set_connected(False)
                print(f"[MJPEGStream] cannot open {self.url}, retrying in {backoff:.1f}s")
                if self._stop_event.wait(backoff):
                    break
                backoff = min(backoff * 2, self.retry_backoff_max_s)
                continue

            print(f"[MJPEGStream] connected to {self.url}")
            self._set_connected(True)
            backoff = self.retry_backoff_base_s

            while not self._stop_event.is_set():
                ok, frame = cap.read()
                if not ok:
                    print("[MJPEGStream] stream dropped, reconnecting")
                    break
                with self._lock:
                    self._frame = frame
                    self._timestamp = time.monotonic()

            cap.release()
            self._set_connected(False)

    def __enter__(self) -> "MJPEGStream":
        return self.start()

    def __exit__(self, *exc_info) -> None:
        self.stop()
