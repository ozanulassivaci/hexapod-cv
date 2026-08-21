from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Detection:
    """A single detected object, backend-agnostic (HSV, YOLO, ...)."""

    label: str
    cx: float  # normalized x, range [-1, 1], 0 = frame center, +1 = right edge
    cy: float  # normalized y, range [-1, 1], 0 = frame center, +1 = bottom edge
    bbox: tuple[int, int, int, int]  # (x, y, w, h) in pixels
    confidence: float


class Detector(ABC):
    """Common interface so the detection backend can be swapped without
    touching main.py or the stream/config layers."""

    @abstractmethod
    def detect(self, frame: np.ndarray) -> list[Detection]:
        raise NotImplementedError


def normalize_center(
    bbox: tuple[int, int, int, int], frame_width: int, frame_height: int
) -> tuple[float, float]:
    """Map a pixel bbox's center to [-1, 1] coordinates centered on the frame."""
    x, y, w, h = bbox
    px, py = x + w / 2, y + h / 2
    cx = (px - frame_width / 2) / (frame_width / 2)
    cy = (py - frame_height / 2) / (frame_height / 2)
    return cx, cy
