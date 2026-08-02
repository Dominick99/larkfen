from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from drone_protocol import PositionSetpoint, VehicleState, WrenchCommand


def _rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclass(frozen=True)
class QuadcopterFlightController:
    mass: float = 1.0
    max_force: float = 22.0
    max_torque: float = 5.0
    linear_drag: float = 0.18
    angular_drag: float = 0.12
    position_gain: float = 2.8
    velocity_gain: float = 2.2
    attitude_gain: float = 7.0
    angular_gain: float = 2.4

    def update(self, state: VehicleState, target: PositionSetpoint) -> WrenchCommand:
        desired_accel = (
            self.position_gain * (target.position - state.position)
            - self.velocity_gain * state.velocity
            + np.array([0.0, 0.0, 9.81])
        )
        desired_force = self.mass * desired_accel
        force_norm = float(np.linalg.norm(desired_force))
        force = desired_force * min(1.0, self.max_force / max(force_norm, 1e-6))

        yaw = math.atan2(
            target.position[1] - state.position[1],
            target.position[0] - state.position[0],
        )
        desired_z = force / max(np.linalg.norm(force), 1e-6)
        desired_x_hint = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        desired_y = np.cross(desired_z, desired_x_hint)
        desired_y /= max(np.linalg.norm(desired_y), 1e-6)
        desired_x = np.cross(desired_y, desired_z)
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))
        rotation = _rotation_matrix(state.orientation_xyzw)
        error_matrix = 0.5 * (
            desired_rotation.T @ rotation - rotation.T @ desired_rotation
        )
        attitude_error = np.array(
            [error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]]
        )
        torque = (
            -self.attitude_gain * attitude_error
            - self.angular_gain * state.angular_velocity
        )
        torque = np.clip(torque, -self.max_torque, self.max_torque)
        return WrenchCommand(
            world_force=force - self.linear_drag * state.velocity,
            body_torque=torque,
            world_torque=-self.angular_drag * state.angular_velocity,
            timestamp_seconds=state.timestamp_seconds,
        )


@dataclass(frozen=True)
class FixedWingFlightController:
    mass: float = 1.8
    wing_area: float = 0.45
    cruise_speed: float = 8.0
    max_thrust: float = 18.0
    lift_coefficient: float = 1.0
    drag_coefficient: float = 0.055
    turn_gain: float = 1.8
    attitude_gain: float = 5.0
    angular_gain: float = 1.8
    max_pitch_degrees: float = 18.0
    air_density: float = 1.225

    def update(self, state: VehicleState, target: PositionSetpoint) -> WrenchCommand:
        rotation = _rotation_matrix(state.orientation_xyzw)
        right = rotation[:, 1]
        airspeed = max(float(np.linalg.norm(state.velocity)), 0.1)
        direction = target.position - state.position
        direction /= max(float(np.linalg.norm(direction)), 1e-6)
        limit = math.sin(math.radians(self.max_pitch_degrees))
        direction[2] = np.clip(direction[2], -limit, limit)
        direction /= max(float(np.linalg.norm(direction)), 1e-6)
        desired_velocity = self.cruise_speed * direction
        navigation_force = self.mass * self.turn_gain * (desired_velocity - state.velocity)
        navigation_force *= min(
            1.0, self.max_thrust / max(float(np.linalg.norm(navigation_force)), 1e-6)
        )
        lift_force = np.array([0.0, 0.0, self.mass * 9.81 * self.lift_coefficient])

        desired_up = np.array([0.0, 0.0, 1.0])
        desired_up -= direction * float(np.dot(desired_up, direction))
        desired_up /= max(float(np.linalg.norm(desired_up)), 1e-6)
        desired_right = np.cross(desired_up, direction)
        desired_rotation = np.column_stack((direction, desired_right, desired_up))
        error_matrix = 0.5 * (
            desired_rotation.T @ rotation - rotation.T @ desired_rotation
        )
        attitude_error = np.array(
            [error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]]
        )
        torque = np.clip(
            -self.attitude_gain * attitude_error
            - self.angular_gain * state.angular_velocity,
            -4.0,
            4.0,
        )
        drag = (
            0.5
            * self.air_density
            * airspeed**2
            * self.wing_area
            * self.drag_coefficient
        )
        velocity_direction = state.velocity / airspeed
        lateral_speed = float(np.dot(state.velocity, right))
        force = (
            navigation_force
            + lift_force
            - drag * velocity_direction
            - 1.8 * lateral_speed * right
        )
        return WrenchCommand(force, torque, np.zeros(3), state.timestamp_seconds)
