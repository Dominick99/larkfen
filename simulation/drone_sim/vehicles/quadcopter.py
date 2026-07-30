from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pybullet as p

from .base import Drone


@dataclass(frozen=True)
class QuadcopterConfig:
    mass: float = 1.0
    arm_length: float = 0.28
    max_force: float = 22.0
    max_torque: float = 5.0
    linear_drag: float = 0.18
    angular_drag: float = 0.12
    position_gain: float = 2.8
    velocity_gain: float = 2.2
    attitude_gain: float = 7.0
    angular_gain: float = 2.4


class Quadcopter(Drone):
    name = "quadcopter"
    waypoint_radius = 0.35

    def __init__(self, config: QuadcopterConfig | None = None) -> None:
        self._config = config or QuadcopterConfig()
        self.body_id = -1

    @property
    def config(self) -> QuadcopterConfig:
        return self._config

    def create(self) -> int:
        body_half = [0.13, 0.09, 0.045]
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=body_half)
        visuals = [
            p.createVisualShape(
                p.GEOM_BOX, halfExtents=body_half, rgbaColor=[0.12, 0.17, 0.22, 1]
            )
        ]
        positions = [[0, 0, 0]]
        orientations = [[0, 0, 0, 1]]

        arm_half = [self.config.arm_length, 0.018, 0.015]
        arm_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=arm_half, rgbaColor=[0.3, 0.34, 0.38, 1]
        )
        for orientation in (
            [0, 0, 0, 1],
            p.getQuaternionFromEuler([0, 0, math.pi / 2]),
        ):
            visuals.append(arm_visual)
            positions.append([0, 0, 0])
            orientations.append(orientation)

        for x, y, color in [
            (self.config.arm_length, 0, [0.9, 0.2, 0.12, 1]),
            (-self.config.arm_length, 0, [0.1, 0.45, 0.9, 1]),
            (0, self.config.arm_length, [0.1, 0.45, 0.9, 1]),
            (0, -self.config.arm_length, [0.1, 0.45, 0.9, 1]),
        ]:
            visuals.append(
                p.createVisualShape(
                    p.GEOM_CYLINDER, radius=0.095, length=0.012, rgbaColor=color
                )
            )
            positions.append([x, y, 0.025])
            orientations.append([0, 0, 0, 1])

        self.body_id = p.createMultiBody(
            baseMass=self.config.mass,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=-1,
            basePosition=[0, 0, 0.7],
            linkMasses=[0] * len(visuals),
            linkCollisionShapeIndices=[-1] * len(visuals),
            linkVisualShapeIndices=visuals,
            linkPositions=positions,
            linkOrientations=orientations,
            linkInertialFramePositions=[[0, 0, 0]] * len(visuals),
            linkInertialFrameOrientations=[[0, 0, 0, 1]] * len(visuals),
            linkParentIndices=[0] * len(visuals),
            linkJointTypes=[p.JOINT_FIXED] * len(visuals),
            linkJointAxis=[[0, 0, 0]] * len(visuals),
        )
        p.changeDynamics(
            self.body_id,
            -1,
            linearDamping=0,
            angularDamping=0,
            lateralFriction=0.8,
            restitution=0.1,
        )
        return self.body_id

    def control(self, target: np.ndarray, time_step: float) -> None:
        del time_step
        position, quaternion, velocity, angular_velocity = self.state()
        desired_accel = (
            self.config.position_gain * (target - position)
            - self.config.velocity_gain * velocity
            + np.array([0.0, 0.0, 9.81])
        )
        desired_force = self.config.mass * desired_accel
        force_norm = float(np.linalg.norm(desired_force))
        force = desired_force * min(
            1.0, self.config.max_force / max(force_norm, 1e-6)
        )

        yaw = math.atan2(target[1] - position[1], target[0] - position[0])
        desired_z = force / max(np.linalg.norm(force), 1e-6)
        desired_x_hint = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        desired_y = np.cross(desired_z, desired_x_hint)
        desired_y /= max(np.linalg.norm(desired_y), 1e-6)
        desired_x = np.cross(desired_y, desired_z)
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))

        rotation = np.asarray(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
        error_matrix = 0.5 * (
            desired_rotation.T @ rotation - rotation.T @ desired_rotation
        )
        attitude_error = np.array(
            [error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]]
        )
        torque = (
            -self.config.attitude_gain * attitude_error
            - self.config.angular_gain * angular_velocity
        )
        torque = np.clip(torque, -self.config.max_torque, self.config.max_torque)

        p.applyExternalForce(
            self.body_id, -1, force.tolist(), position.tolist(), p.WORLD_FRAME
        )
        p.applyExternalTorque(
            self.body_id, -1, torque.tolist(), p.LINK_FRAME
        )
        p.applyExternalForce(
            self.body_id,
            -1,
            (-self.config.linear_drag * velocity).tolist(),
            position.tolist(),
            p.WORLD_FRAME,
        )
        p.applyExternalTorque(
            self.body_id,
            -1,
            (-self.config.angular_drag * angular_velocity).tolist(),
            p.WORLD_FRAME,
        )

    def default_waypoints(self) -> np.ndarray:
        return np.array(
            [
                [0, 0, 1.2],
                [2.0, 0, 1.8],
                [2.2, 2.0, 2.2],
                [0, 2.2, 1.5],
                [-1.5, 0, 1.8],
            ],
            dtype=float,
        )

