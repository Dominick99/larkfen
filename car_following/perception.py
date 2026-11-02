"""Simulator-independent RGB detection and short-lived target identity.

Replace Detector.detect with a learned detector without changing guidance.
Boxes use pixel xyxy coordinates in the unmodified RGB camera frame.
"""
from dataclasses import dataclass
from typing import Protocol

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    box: tuple[float, float, float, float]
    label: str
    confidence: float


@dataclass(frozen=True)
class Track:
    detection: Detection | None
    track_id: int | None
    state: str


class Detector(Protocol):
    def detect(self, rgb: np.ndarray) -> list[Detection]: ...


class ColorDetector:
    """Prototype ATR by color, not a learned semantic car classifier."""
    def detect(self, rgb):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        ranges = {'green': [(35, 85)], 'blue': [(90, 135)], 'red': [(0, 12), (165, 179)]}
        detections = []
        for label, bands in ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for low, high in bands:
                mask |= cv2.inRange(hsv, (low, 95, 45), (high, 255, 255))
            count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
            for x, y, w, h, area in stats[1:count]:
                if area >= 12 and w >= 3 and h >= 3:
                    detections.append(Detection((float(x), float(y), float(x+w), float(y+h)), label, 1.0))
        return detections


class GreenTracker:
    """Select green detections; associate by nearest image center.

    Keeps identity through short gaps, expires it after 3 s, and never emits a
    stale box as a current measurement. Multiple identical green cars are outside
    this prototype's identity guarantees.
    """
    def __init__(self):
        self.last = None
        self.last_seen = -1e9
        self.track_id = None
        self.counter = 0

    def update(self, detections, time):
        candidates = [d for d in detections if d.label == 'green' and d.confidence >= .5]
        if time - self.last_seen > 3:
            self.last = None
            self.track_id = None
        if not candidates:
            return Track(None, self.track_id, 'lost' if self.track_id else 'searching')
        def center(d):
            x1, y1, x2, y2 = d.box
            return np.array([(x1+x2)/2, (y1+y2)/2])
        chosen = min(candidates, key=lambda d: np.linalg.norm(center(d)-center(self.last))) if self.last else max(candidates, key=lambda d: (d.box[2]-d.box[0])*(d.box[3]-d.box[1]))
        if self.track_id is None:
            self.counter += 1
            self.track_id = self.counter
        self.last, self.last_seen = chosen, time
        return Track(chosen, self.track_id, 'tracking')


def iou(a, b):
    if a is None or b is None:
        return 0.0
    intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
    return intersection / union if union else 0.0
