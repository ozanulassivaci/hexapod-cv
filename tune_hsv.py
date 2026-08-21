"""Live HSV range tuner. Adjust the trackbars until the mask isolates the
target object, then press 'q' to print the values ready to paste into
config.py's HSVRange.
"""

import cv2

from config import load_config
from stream.mjpeg_stream import MJPEGStream

WINDOW = "tune_hsv"
TRACKBARS = [
    ("H min", "h_min", 179),
    ("H max", "h_max", 179),
    ("S min", "s_min", 255),
    ("S max", "s_max", 255),
    ("V min", "v_min", 255),
    ("V max", "v_max", 255),
]


def build_trackbars(initial: dict[str, int]) -> None:
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    for label, key, max_value in TRACKBARS:
        cv2.createTrackbar(label, WINDOW, initial[key], max_value, lambda _: None)


def read_trackbars() -> dict[str, int]:
    return {
        key: cv2.getTrackbarPos(label, WINDOW) for label, key, _ in TRACKBARS
    }


def main() -> None:
    config = load_config()
    lower, upper = config.detection.hsv.lower, config.detection.hsv.upper
    initial = {
        "h_min": lower[0], "s_min": lower[1], "v_min": lower[2],
        "h_max": upper[0], "s_max": upper[1], "v_max": upper[2],
    }
    build_trackbars(initial)

    stream = MJPEGStream(
        config.stream.url,
        retry_backoff_base_s=config.stream.retry_backoff_base_s,
        retry_backoff_max_s=config.stream.retry_backoff_max_s,
    ).start()

    try:
        values = initial
        while True:
            frame, _ = stream.read()
            if frame is None:
                if cv2.waitKey(50) & 0xFF in (ord("q"), 27):
                    break
                continue

            values = read_trackbars()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(
                hsv,
                (values["h_min"], values["s_min"], values["v_min"]),
                (values["h_max"], values["s_max"], values["v_max"]),
            )
            masked = cv2.bitwise_and(frame, frame, mask=mask)
            cv2.imshow(WINDOW, cv2.hconcat([frame, masked]))

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        stream.stop()
        cv2.destroyAllWindows()

    lower = (values["h_min"], values["s_min"], values["v_min"])
    upper = (values["h_max"], values["s_max"], values["v_max"])
    print("\nPaste into config.py:")
    print(f"    lower: tuple[int, int, int] = {lower}")
    print(f"    upper: tuple[int, int, int] = {upper}")


if __name__ == "__main__":
    main()
