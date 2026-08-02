from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pybullet as p


@dataclass(frozen=True)
class BoundingBox:
    x_min: int
    y_min: int
    x_max: int
    y_max: int
    image_width: int
    image_height: int

    @property
    def center_error(self) -> tuple[float, float]:
        x = (self.x_min + self.x_max) / self.image_width - 1.0
        y = (self.y_min + self.y_max) / self.image_height - 1.0
        return x, y

    @property
    def area_fraction(self) -> float:
        return (
            (self.x_max - self.x_min + 1)
            * (self.y_max - self.y_min + 1)
            / (self.image_width * self.image_height)
        )

    def as_dict(self) -> dict[str, float | int]:
        return {
            "x_min": self.x_min,
            "y_min": self.y_min,
            "x_max": self.x_max,
            "y_max": self.y_max,
            "center_error_x": self.center_error[0],
            "center_error_y": self.center_error[1],
            "area_fraction": self.area_fraction,
        }


def detect_body(mask: np.ndarray, body_id: int) -> BoundingBox | None:
    # PyBullet stores the object id in the low 24 bits of its segmentation value.
    pixels = (mask & ((1 << 24) - 1)) == body_id
    ys, xs = np.nonzero(pixels)
    if not len(xs):
        return None
    height, width = mask.shape
    return BoundingBox(
        int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()), width, height
    )


def draw_bounding_box(
    image: np.ndarray, box: BoundingBox | None, thickness: int = 3
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
