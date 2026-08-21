import cv2
import numpy as np

from perception.detection import Detection, Detector, normalize_center


class HSVDetector(Detector):
    """Threshold-based detector: masks a frame by an HSV range and turns
    surviving contours above a minimum area into Detections."""

    def __init__(
        self,
        lower: tuple[int, int, int],
        upper: tuple[int, int, int],
        min_contour_area: int,
        label: str = "object",
    ):
        self.lower = np.array(lower, dtype=np.uint8)
        self.upper = np.array(upper, dtype=np.uint8)
        self.min_contour_area = min_contour_area
        self.label = label

    def detect(self, frame: np.ndarray) -> list[Detection]:
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue
            bbox = cv2.boundingRect(contour)
            cx, cy = normalize_center(bbox, width, height)
            # No probabilistic confidence for a threshold mask; contours that
            # pass the area filter are reported as fully confident.
            detections.append(
                Detection(
                    label=self.label,
                    cx=cx,
                    cy=cy,
                    bbox=bbox,
                    confidence=1.0,
                )
            )
        return detections
