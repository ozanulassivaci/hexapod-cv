import time

import cv2
import numpy as np

from config import load_config
from perception.hsv_detector import HSVDetector
from stream.mjpeg_stream import MJPEGStream


def draw_detections(frame, detections) -> None:
    for det in detections:
        x, y, w, h = det.bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        label = f"{det.label} ({det.cx:+.2f}, {det.cy:+.2f})"
        cv2.putText(
            frame, label, (x, max(y - 8, 0)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA,
        )


def draw_overlay(frame, fps: float, frame_age_ms: float | None) -> None:
    age_text = f"{frame_age_ms:.0f}ms" if frame_age_ms is not None else "n/a"
    cv2.putText(
        frame, f"FPS: {fps:.1f}  age: {age_text}", (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA,
    )


def main() -> None:
    config = load_config()

    detector = HSVDetector(
        lower=config.detection.hsv.lower,
        upper=config.detection.hsv.upper,
        min_contour_area=config.detection.min_contour_area,
        label=config.detection.label,
    )

    stream = MJPEGStream(
        config.stream.url,
        retry_backoff_base_s=config.stream.retry_backoff_base_s,
        retry_backoff_max_s=config.stream.retry_backoff_max_s,
    ).start()

    cv2.namedWindow(config.window.title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(config.window.title, config.window.width, config.window.height)

    last_loop_time = time.monotonic()
    fps = 0.0

    try:
        while True:
            frame, timestamp = stream.read()
            now = time.monotonic()

            dt = now - last_loop_time
            last_loop_time = now
            if dt > 0:
                fps = fps * 0.9 + (1.0 / dt) * 0.1

            if frame is None:
                placeholder = np.zeros(
                    (config.window.height, config.window.width, 3), dtype="uint8"
                )
                cv2.putText(
                    placeholder, "waiting for stream...", (16, config.window.height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA,
                )
                cv2.imshow(config.window.title, placeholder)
            else:
                frame_age_ms = (now - timestamp) * 1000 if timestamp else None
                detections = detector.detect(frame)
                draw_detections(frame, detections)
                draw_overlay(frame, fps, frame_age_ms)
                cv2.imshow(config.window.title, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        stream.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
