from __future__ import annotations

import math

import numpy as np
import pybullet as p

from robotics.contracts import VehicleState
from robotics.robot_models.quadcopter import (
    QuadcopterConfig,
    RotorModel,
    rotor_positions,
)


class Quadcopter:
    """Thin PyBullet adapter for the portable quadcopter dynamics model."""

    _spin_directions = np.array([-1.0, 1.0, -1.0, 1.0])

    def __init__(self, config: QuadcopterConfig | None = None) -> None:
        self.config = config or QuadcopterConfig()
        self.rotors = RotorModel(self.config)
        self.body_id = -1
        self.rotor_positions = rotor_positions(self.config)

    def create(self, position: tuple[float, float, float] = (0.0, 0.0, 1.0)) -> int:
        body_half_extents = [0.11, 0.08, 0.04]
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=body_half_extents)
        body_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=body_half_extents,
            rgbaColor=[0.12, 0.16, 0.2, 1.0],
        )
        arm_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.config.arm_length_m, 0.015, 0.012],
            rgbaColor=[0.35, 0.38, 0.42, 1.0],
        )
        rotor_visuals = [
            p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.075,
                length=0.012,
                rgbaColor=[0.95, 0.25, 0.12, 1.0]
                if index == 0
                else [0.15, 0.45, 0.9, 1.0],
            )
            for index in range(4)
        ]
        visual_shapes = [body_visual, arm_visual, arm_visual, *rotor_visuals]
        visual_positions = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            *self.rotor_positions.tolist(),
        ]
        visual_orientations = [
            [0.0, 0.0, 0.0, 1.0],
            p.getQuaternionFromEuler([0.0, 0.0, math.pi / 4.0]),
            p.getQuaternionFromEuler([0.0, 0.0, -math.pi / 4.0]),
            *[[0.0, 0.0, 0.0, 1.0] for _ in range(4)],
        ]
        self.body_id = p.createMultiBody(
            baseMass=self.config.mass_kg,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=-1,
            basePosition=position,
            linkMasses=[0.0] * len(visual_shapes),
            linkCollisionShapeIndices=[-1] * len(visual_shapes),
            linkVisualShapeIndices=visual_shapes,
            linkPositions=visual_positions,
            linkOrientations=visual_orientations,
            linkInertialFramePositions=[[0.0, 0.0, 0.0]] * len(visual_shapes),
            linkInertialFrameOrientations=[[0.0, 0.0, 0.0, 1.0]]
            * len(visual_shapes),
            linkParentIndices=[0] * len(visual_shapes),
            linkJointTypes=[p.JOINT_FIXED] * len(visual_shapes),
            linkJointAxis=[[0.0, 0.0, 0.0]] * len(visual_shapes),
        )
        p.changeDynamics(
            self.body_id,
            -1,
            localInertiaDiagonal=self.config.inertia_diagonal_kg_m2,
            linearDamping=0.0,
            angularDamping=0.0,
        )
        self.rotors.reset()
        return self.body_id

    def state(self) -> VehicleState:
        position, orientation = p.getBasePositionAndOrientation(self.body_id)
        linear_velocity, angular_velocity = p.getBaseVelocity(self.body_id)
        return VehicleState(
            position=np.asarray(position, dtype=float),
            orientation_xyzw=np.asarray(orientation, dtype=float),
            linear_velocity=np.asarray(linear_velocity, dtype=float),
            angular_velocity=np.asarray(angular_velocity, dtype=float),
        )

    def apply_motor_command(self, throttle: np.ndarray, timestep: float) -> None:
        thrusts = self.rotors.step(throttle, timestep)
        for rotor_position, thrust in zip(self.rotor_positions, thrusts):
            p.applyExternalForce(
                self.body_id,
                -1,
                [0.0, 0.0, float(thrust)],
                rotor_position.tolist(),
                p.LINK_FRAME,
            )
        yaw_torque = float(
            np.dot(self._spin_directions, thrusts)
            * self.config.yaw_torque_per_thrust_m
        )
        state = self.state()
        rotation = np.asarray(
            p.getMatrixFromQuaternion(state.orientation_xyzw)
        ).reshape(3, 3)
        body_angular_velocity = rotation.T @ state.angular_velocity
        p.applyExternalTorque(
            self.body_id,
            -1,
            [0.0, 0.0, yaw_torque],
            p.LINK_FRAME,
        )
        p.applyExternalForce(
            self.body_id,
            -1,
            (-self.config.linear_drag_n_per_mps * state.linear_velocity).tolist(),
            state.position.tolist(),
            p.WORLD_FRAME,
        )
        p.applyExternalTorque(
            self.body_id,
            -1,
            (-self.config.angular_drag_nm_per_rad_s * body_angular_velocity).tolist(),
            p.LINK_FRAME,
        )
