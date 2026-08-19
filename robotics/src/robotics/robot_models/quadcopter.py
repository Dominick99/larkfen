from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class QuadcopterConfig:
    """Physical and actuator parameters for an X-configuration quadcopter."""

    mass_kg: float = 1.0
    arm_length_m: float = 0.24
    max_thrust_per_rotor_n: float = 6.0
    yaw_torque_per_thrust_m: float = 0.018
    motor_time_constant_s: float = 0.035
    linear_drag_n_per_mps: float = 0.12
    angular_drag_nm_per_rad_s: float = 0.025
    inertia_diagonal_kg_m2: tuple[float, float, float] = (0.015, 0.015, 0.025)


class RotorModel:
    """Simulator-independent motor lag and thrust calculation."""

    def __init__(self, config: QuadcopterConfig) -> None:
        self.config = config
        self.throttle = np.zeros(4, dtype=float)

    def reset(self) -> None:
        self.throttle.fill(0.0)

    def step(self, desired_throttle: np.ndarray, timestep: float) -> np.ndarray:
        desired = np.clip(np.asarray(desired_throttle, dtype=float), 0.0, 1.0)
        response = min(timestep / self.config.motor_time_constant_s, 1.0)
        self.throttle += response * (desired - self.throttle)
        return self.config.max_thrust_per_rotor_n * self.throttle**2


def rotor_positions(config: QuadcopterConfig) -> np.ndarray:
    """Return X-configuration rotor positions in the body frame."""
    offset = config.arm_length_m / np.sqrt(2.0)
    return np.array(
        [
            [offset, offset, 0.03],
            [-offset, offset, 0.03],
            [-offset, -offset, 0.03],
            [offset, -offset, 0.03],
        ]
    )


def allocate_rotor_throttle(
    collective_thrust_n: float,
    body_torque_nm: np.ndarray,
    config: QuadcopterConfig,
) -> np.ndarray:
    """Allocate desired collective thrust and body torque across four rotors."""
    positions = rotor_positions(config)
    spin_directions = np.array([-1.0, 1.0, -1.0, 1.0])
    allocation = np.vstack(
        (
            np.ones(4),
            positions[:, 1],
            -positions[:, 0],
            config.yaw_torque_per_thrust_m * spin_directions,
        )
    )
    requested_wrench = np.array([collective_thrust_n, *body_torque_nm])
    rotor_thrust = np.linalg.solve(allocation, requested_wrench)
    rotor_thrust = np.clip(rotor_thrust, 0.0, config.max_thrust_per_rotor_n)
    return np.sqrt(rotor_thrust / config.max_thrust_per_rotor_n)
