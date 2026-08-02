from __future__ import annotations

import math

import numpy as np
import pybullet as p

from .perception import BoundingBox


class BoundingBoxPursuitController:
    """Turns image-space box measurements into collision-course waypoints."""

    def __init__(self, cruise_altitude: float = 2.5) -> None:
        self.cruise_altitude = cruise_altitude
        self.last_seen_heading: float | None = None
        self.last_forward = 1.0

    def waypoint(
        self, drone_body_id: int, box: BoundingBox | None
    ) -> np.ndarray:
        position, orientation = p.getBasePositionAndOrientation(drone_body_id)
        position = np.asarray(position, dtype=float)
        yaw = p.getEulerFromQuaternion(orientation)[2]

        if box is not None:
            horizontal_error, _ = box.center_error
            # The camera's horizontal FOV is 72 degrees. Point toward the box.
            heading = yaw - horizontal_error * math.radians(36)
            self.last_seen_heading = heading
            # Close aggressively while slowing slightly for large horizontal
            # errors so the target stays in view during the turn.
            forward = float(np.clip(3.2 - 2.5 * abs(horizontal_error), 1.0, 3.2))
            self.last_forward = forward
            # Apparent box growth is the only range cue used to begin the dive.
            # At close range the waypoint is driven through the car's height.
            desired_altitude = float(
                np.interp(box.area_fraction, [0.008, 0.055], [self.cruise_altitude, 0.32])
            )
        else:
            # Continue briefly in the last observed direction to reacquire the car.
            heading = self.last_seen_heading if self.last_seen_heading is not None else yaw
            forward = self.last_forward
            desired_altitude = self.cruise_altitude

        return position + np.array(
            [forward * math.cos(heading), forward * math.sin(heading), 0.0]
        ) * np.array([1.0, 1.0, 0.0]) + np.array(
            [0.0, 0.0, desired_altitude - position[2]]
        )
