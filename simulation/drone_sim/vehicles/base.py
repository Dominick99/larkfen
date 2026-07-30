from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict
from typing import Any

import numpy as np
import pybullet as p


class Drone(ABC):
    """Common interface implemented by every simulated airframe."""

    name: str
    body_id: int
    waypoint_radius: float

    @property
    @abstractmethod
    def config(self) -> Any:
        """Return the vehicle-specific configuration dataclass."""

    @abstractmethod
    def create(self) -> int:
        """Create the vehicle in the active PyBullet world."""

    @abstractmethod
    def control(self, target: np.ndarray, time_step: float) -> None:
        """Apply forces and torques that guide the vehicle toward a target."""

    @abstractmethod
    def default_waypoints(self) -> np.ndarray:
        """Return a course suited to this airframe."""

    def state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        position, orientation = p.getBasePositionAndOrientation(self.body_id)
        velocity, angular_velocity = p.getBaseVelocity(self.body_id)
        return tuple(
            np.asarray(value, dtype=float)
            for value in (position, orientation, velocity, angular_velocity)
        )

    def config_dict(self) -> dict[str, Any]:
        return asdict(self.config)

