from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pybullet as p

from .base import Drone


@dataclass(frozen=True)
class FixedWingConfig:
    mass: float = 1.8
    wing_span: float = 1.4
    wing_area: float = 0.45
    cruise_speed: float = 8.0
    max_thrust: float = 18.0
    lift_coefficient: float = 1.0
    drag_coefficient: float = 0.055
    turn_gain: float = 1.8
    attitude_gain: float = 5.0
    angular_gain: float = 1.8
    max_pitch_degrees: float = 18.0


class FixedWing(Drone):
    """Simplified aerodynamic fixed-wing model with waypoint autopilot."""

    name = "fixed-wing"
    waypoint_radius = 1.5
    _air_density = 1.225

    def __init__(self, config: FixedWingConfig | None = None) -> None:
        self._config = config or FixedWingConfig()
        self.body_id = -1

    @property
    def config(self) -> FixedWingConfig:
        return self._config

    def create(self) -> int:
        fuselage_half = [0.42, 0.065, 0.07]
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=fuselage_half)
        visual_specs = [
            (p.GEOM_BOX, fuselage_half, [0.12, 0.22, 0.32, 1], [0, 0, 0]),
            (
                p.GEOM_BOX,
                [0.13, self.config.wing_span / 2, 0.018],
                [0.86, 0.88, 0.9, 1],
                [-0.03, 0, 0],
            ),
            (
                p.GEOM_BOX,
                [0.11, 0.28, 0.012],
                [0.86, 0.88, 0.9, 1],
                [-0.35, 0, 0.04],
            ),
            (
                p.GEOM_BOX,
                [0.1, 0.015, 0.16],
                [0.9, 0.22, 0.12, 1],
                [-0.37, 0, 0.12],
            ),
        ]
        visuals, positions = [], []
        for shape_type, half_extents, color, position in visual_specs:
            visuals.append(
                p.createVisualShape(
                    shape_type, halfExtents=half_extents, rgbaColor=color
                )
            )
            positions.append(position)

        self.body_id = p.createMultiBody(
            baseMass=self.config.mass,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=-1,
            basePosition=[-4, 0, 2.2],
            linkMasses=[0] * len(visuals),
            linkCollisionShapeIndices=[-1] * len(visuals),
            linkVisualShapeIndices=visuals,
            linkPositions=positions,
            linkOrientations=[[0, 0, 0, 1]] * len(visuals),
            linkInertialFramePositions=[[0, 0, 0]] * len(visuals),
            linkInertialFrameOrientations=[[0, 0, 0, 1]] * len(visuals),
            linkParentIndices=[0] * len(visuals),
            linkJointTypes=[p.JOINT_FIXED] * len(visuals),
            linkJointAxis=[[0, 0, 0]] * len(visuals),
        )
        p.changeDynamics(
            self.body_id, -1, linearDamping=0, angularDamping=0, restitution=0.1
        )
        p.resetBaseVelocity(self.body_id, linearVelocity=[self.config.cruise_speed, 0, 0])
        return self.body_id

    def control(self, target: np.ndarray, time_step: float) -> None:
        del time_step
        position, quaternion, velocity, angular_velocity = self.state()
        rotation = np.asarray(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
        right = rotation[:, 1]
        airspeed = max(float(np.linalg.norm(velocity)), 0.1)

        path = target - position
        direction = path / max(float(np.linalg.norm(path)), 1e-6)
        direction[2] = np.clip(
            direction[2], -math.sin(math.radians(self.config.max_pitch_degrees)),
            math.sin(math.radians(self.config.max_pitch_degrees)),
        )
        direction /= max(float(np.linalg.norm(direction)), 1e-6)

        # Maintain forward airspeed at all times; unlike the quadcopter this
        # controller never commands a stationary hover.
        desired_velocity = self.config.cruise_speed * direction
        navigation_force = (
            self.config.mass
            * self.config.turn_gain
            * (desired_velocity - velocity)
        )
        navigation_force *= min(
            1.0,
            self.config.max_thrust
            / max(float(np.linalg.norm(navigation_force)), 1e-6),
        )
        lift_force = np.array(
            [0.0, 0.0, self.config.mass * 9.81 * self.config.lift_coefficient]
        )
        force = navigation_force + lift_force

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
        torque = (
            -self.config.attitude_gain * attitude_error
            - self.config.angular_gain * angular_velocity
        )
        torque = np.clip(torque, -4.0, 4.0)

        p.applyExternalForce(
            self.body_id,
            -1,
            force.tolist(),
            position.tolist(),
            p.WORLD_FRAME,
        )
        p.applyExternalTorque(self.body_id, -1, torque.tolist(), p.LINK_FRAME)
        # Parasite drag and vertical-tail side-force damping provide basic
        # fixed-wing aerodynamic behavior without a full CFD model.
        drag = (
            0.5
            * self._air_density
            * airspeed**2
            * self.config.wing_area
            * self.config.drag_coefficient
        )
        velocity_direction = velocity / airspeed
        p.applyExternalForce(
            self.body_id,
            -1,
            (-drag * velocity_direction).tolist(),
            position.tolist(),
            p.WORLD_FRAME,
        )
        lateral_speed = float(np.dot(velocity, right))
        p.applyExternalForce(
            self.body_id,
            -1,
            (-1.8 * lateral_speed * right).tolist(),
            position.tolist(),
            p.WORLD_FRAME,
        )

    def default_waypoints(self) -> np.ndarray:
        return np.array(
            [
                [5, 0, 2.5],
                [5, 6, 3.2],
                [-5, 6, 3.8],
                [-6, -4, 3.0],
                [5, -5, 2.6],
            ],
            dtype=float,
        )
