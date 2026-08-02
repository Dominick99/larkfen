from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict
from typing import Any

import numpy as np
import pybullet as p

from drone_protocol import VehicleState, WrenchCommand


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
    def default_waypoints(self) -> np.ndarray:
        """Return a course suited to this airframe."""

    def state(self, timestamp_seconds: float = 0.0) -> VehicleState:
        position, orientation = p.getBasePositionAndOrientation(self.body_id)
        velocity, angular_velocity = p.getBaseVelocity(self.body_id)
        return VehicleState(
            position=np.asarray(position, dtype=float),
            orientation_xyzw=np.asarray(orientation, dtype=float),
            velocity=np.asarray(velocity, dtype=float),
            angular_velocity=np.asarray(angular_velocity, dtype=float),
            timestamp_seconds=timestamp_seconds,
        )

    def apply_wrench(self, command: WrenchCommand) -> None:
        position = self.state(command.timestamp_seconds).position
        p.applyExternalForce(
            self.body_id,
            -1,
            command.world_force.tolist(),
            position.tolist(),
            p.WORLD_FRAME,
        )
        p.applyExternalTorque(
            self.body_id, -1, command.body_torque.tolist(), p.LINK_FRAME
        )
        p.applyExternalTorque(
            self.body_id, -1, command.world_torque.tolist(), p.WORLD_FRAME
        )

    def config_dict(self) -> dict[str, Any]:
        return asdict(self.config)

