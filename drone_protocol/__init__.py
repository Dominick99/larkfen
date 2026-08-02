"""Simulator-independent contracts shared by every drone-stack component."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BoundingBoxObservation:
    x_min: int
    y_min: int
    x_max: int
    y_max: int
    image_width: int
    image_height: int
    timestamp_seconds: float
    confidence: float = 1.0

    @property
    def center_error(self) -> tuple[float, float]:
        return (
            (self.x_min + self.x_max) / self.image_width - 1.0,
            (self.y_min + self.y_max) / self.image_height - 1.0,
        )

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
            "timestamp_seconds": self.timestamp_seconds,
            "confidence": self.confidence,
        }


@dataclass(frozen=True)
class VehicleState:
    position: np.ndarray
    orientation_xyzw: np.ndarray
    velocity: np.ndarray
    angular_velocity: np.ndarray
    timestamp_seconds: float


@dataclass(frozen=True)
class PositionSetpoint:
    position: np.ndarray
    timestamp_seconds: float


@dataclass(frozen=True)
class WrenchCommand:
    world_force: np.ndarray
    body_torque: np.ndarray
    world_torque: np.ndarray
    timestamp_seconds: float
