import cv2
import numpy as np

from perception.hsv_detector import HSVDetector

BLUE_BGR = (255, 0, 0)
BLUE_LOWER = (100, 150, 50)
BLUE_UPPER = (140, 255, 255)


def make_frame_with_rect(size=(120, 160), rect=(40, 30, 30, 20), color=BLUE_BGR):
    height, width = size
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    x, y, w, h = rect
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness=-1)
    return frame


def test_detects_single_blue_rectangle():
    rect = (40, 30, 30, 20)
    frame = make_frame_with_rect(rect=rect)
    detector = HSVDetector(BLUE_LOWER, BLUE_UPPER, min_contour_area=50, label="blue")

    detections = detector.detect(frame)

    assert len(detections) == 1
    det = detections[0]
    assert det.label == "blue"
    assert det.confidence == 1.0
    x, y, w, h = det.bbox
    assert abs(x - rect[0]) <= 1
    assert abs(y - rect[1]) <= 1
    assert abs(w - rect[2]) <= 1
    assert abs(h - rect[3]) <= 1


def test_ignores_contours_below_min_area():
    frame = make_frame_with_rect(rect=(10, 10, 3, 3))  # area ~9, tiny
    detector = HSVDetector(BLUE_LOWER, BLUE_UPPER, min_contour_area=500, label="blue")

    detections = detector.detect(frame)

    assert detections == []


def test_ignores_non_matching_color():
    red_bgr = (0, 0, 255)
    frame = make_frame_with_rect(rect=(40, 30, 30, 20), color=red_bgr)
    detector = HSVDetector(BLUE_LOWER, BLUE_UPPER, min_contour_area=50, label="blue")

    detections = detector.detect(frame)

    assert detections == []


def test_detects_multiple_disjoint_regions():
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    cv2.rectangle(frame, (10, 10), (30, 30), BLUE_BGR, thickness=-1)
    cv2.rectangle(frame, (150, 80), (180, 110), BLUE_BGR, thickness=-1)
    detector = HSVDetector(BLUE_LOWER, BLUE_UPPER, min_contour_area=50, label="blue")

    detections = detector.detect(frame)

    assert len(detections) == 2
