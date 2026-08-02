from __future__ import annotations

import math

import numpy as np

from drone_protocol import BoundingBoxObservation, PositionSetpoint, VehicleState


def _yaw_from_quaternion(quaternion: np.ndarray) -> float:
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class BoundingBoxInterceptionGuidance:
    """Rule-based companion guidance with no simulator dependencies."""

    def __init__(self, cruise_altitude: float = 2.5) -> None:
        self.cruise_altitude = cruise_altitude
        self.last_seen_heading: float | None = None
        self.last_forward = 1.0

    def update(
        self,
        observation: BoundingBoxObservation | None,
        vehicle: VehicleState,
    ) -> PositionSetpoint:
        yaw = _yaw_from_quaternion(vehicle.orientation_xyzw)
        if observation is not None:
            horizontal_error, _ = observation.center_error
            heading = yaw - horizontal_error * math.radians(36)
            self.last_seen_heading = heading
            forward = float(np.clip(3.2 - 2.5 * abs(horizontal_error), 1.0, 3.2))
            self.last_forward = forward
            desired_altitude = float(
                np.interp(
                    observation.area_fraction,
                    [0.008, 0.055],
                    [self.cruise_altitude, 0.32],
                )
            )
        else:
            heading = self.last_seen_heading if self.last_seen_heading is not None else yaw
            forward = self.last_forward
            desired_altitude = self.cruise_altitude

        position = vehicle.position + np.array(
            [forward * math.cos(heading), forward * math.sin(heading), 0.0]
        )
        position[2] = desired_altitude
        return PositionSetpoint(position, vehicle.timestamp_seconds)
