from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VehicleState:
    """Canonical rigid-body state shared by controllers and backends."""

    position: np.ndarray
    orientation_xyzw: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray


@dataclass(frozen=True)
class VelocityCommand:
    """A body-relative movement command suitable for policies and autopilots."""

    forward_mps: float = 0.0
    right_mps: float = 0.0
    up_mps: float = 0.0
    yaw_rate_rad_s: float = 0.0
