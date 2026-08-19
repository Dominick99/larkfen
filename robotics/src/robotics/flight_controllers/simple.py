from __future__ import annotations

import math

import numpy as np

from robotics.contracts import VehicleState, VelocityCommand
from robotics.robot_models.quadcopter import (
    QuadcopterConfig,
    allocate_rotor_throttle,
)


def rotation_matrix_xyzw(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """Convert an XYZW quaternion into a body-to-world rotation matrix."""
    x, y, z, w = quaternion_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class SimpleVelocityFlightController:
    """Convert a body-relative velocity command into four rotor commands."""

    def __init__(self, config: QuadcopterConfig) -> None:
        self.config = config
        self.desired_yaw: float | None = None
        self.velocity_gain = 2.0
        self.attitude_gain = np.array([0.11, 0.11, 0.07])
        self.angular_rate_gain = np.array([0.035, 0.035, 0.025])

    def reset(self) -> None:
        self.desired_yaw = None

    def update(
        self,
        state: VehicleState,
        command: VelocityCommand,
        timestep: float,
    ) -> np.ndarray:
        rotation = rotation_matrix_xyzw(state.orientation_xyzw)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        if self.desired_yaw is None:
            self.desired_yaw = yaw
        self.desired_yaw += command.yaw_rate_rad_s * timestep

        horizontal_rotation = np.array(
            [
                [math.cos(yaw), -math.sin(yaw)],
                [math.sin(yaw), math.cos(yaw)],
            ]
        )
        desired_horizontal = horizontal_rotation @ np.array(
            [command.forward_mps, command.right_mps]
        )
        desired_velocity = np.array(
            [desired_horizontal[0], desired_horizontal[1], command.up_mps]
        )
        desired_acceleration = self.velocity_gain * (
            desired_velocity - state.linear_velocity
        ) + np.array([0.0, 0.0, 9.81])
        desired_force = self.config.mass_kg * desired_acceleration

        desired_z = desired_force / max(float(np.linalg.norm(desired_force)), 1e-9)
        heading = np.array(
            [math.cos(self.desired_yaw), math.sin(self.desired_yaw), 0.0]
        )
        desired_y = np.cross(desired_z, heading)
        desired_y /= max(float(np.linalg.norm(desired_y)), 1e-9)
        desired_x = np.cross(desired_y, desired_z)
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))

        error_matrix = 0.5 * (
            desired_rotation.T @ rotation - rotation.T @ desired_rotation
        )
        attitude_error = np.array(
            [error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]]
        )
        body_angular_velocity = rotation.T @ state.angular_velocity
        body_torque = (
            -self.attitude_gain * attitude_error
            - self.angular_rate_gain * body_angular_velocity
        )
        collective_thrust = float(np.dot(desired_force, rotation[:, 2]))
        max_collective = 4.0 * self.config.max_thrust_per_rotor_n
        collective_thrust = float(np.clip(collective_thrust, 0.0, max_collective))
        return allocate_rotor_throttle(
            collective_thrust,
            body_torque,
            self.config,
        )
