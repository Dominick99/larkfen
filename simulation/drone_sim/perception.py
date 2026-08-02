from __future__ import annotations

import numpy as np
import pybullet as p

from drone_protocol import BoundingBoxObservation


def detect_body(
    mask: np.ndarray, body_id: int, timestamp_seconds: float
) -> BoundingBoxObservation | None:
    # PyBullet stores the object id in the low 24 bits of its segmentation value.
    pixels = (mask & ((1 << 24) - 1)) == body_id
    ys, xs = np.nonzero(pixels)
    if not len(xs):
        return None
    height, width = mask.shape
    return BoundingBoxObservation(
        int(xs.min()),
        int(ys.min()),
        int(xs.max()),
        int(ys.max()),
        width,
        height,
        timestamp_seconds,
    )


def draw_bounding_box(
    image: np.ndarray, box: BoundingBoxObservation | None, thickness: int = 3
) -> np.ndarray:
    output = image.copy()
    if box is None:
        return output
    color = np.array([20, 255, 70], dtype=np.uint8)
    x0, y0, x1, y1 = box.x_min, box.y_min, box.x_max, box.y_max
    output[y0 : min(y0 + thickness, y1 + 1), x0 : x1 + 1] = color
    output[max(y1 - thickness + 1, y0) : y1 + 1, x0 : x1 + 1] = color
    output[y0 : y1 + 1, x0 : min(x0 + thickness, x1 + 1)] = color
    output[y0 : y1 + 1, max(x1 - thickness + 1, x0) : x1 + 1] = color
    return output
