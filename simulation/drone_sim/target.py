from __future__ import annotations

import math

import numpy as np
import pybullet as p


class MovingCar:
    """A kinematic car that follows a deterministic looping road course."""

    def __init__(self) -> None:
        self.body_id = -1

    def create(self) -> int:
        half_extents = [0.65, 0.32, 0.22]
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.95, 0.16, 0.06, 1]
        )
        self.body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[5.0, 0.0, 0.24],
        )
        return self.body_id

    def update(self, elapsed: float) -> None:
        # An elongated loop keeps the target moving without leaving the scene.
        omega = 0.16
        phase = omega * elapsed
        position = np.array(
            [5.0 + 3.2 * math.sin(phase), 2.2 * math.sin(2.0 * phase), 0.24]
        )
        velocity = np.array(
            [3.2 * omega * math.cos(phase), 4.4 * omega * math.cos(2.0 * phase), 0]
        )
        yaw = math.atan2(velocity[1], velocity[0])
        p.resetBasePositionAndOrientation(
            self.body_id, position.tolist(), p.getQuaternionFromEuler([0, 0, yaw])
        )
        p.resetBaseVelocity(self.body_id, linearVelocity=velocity.tolist())

    def position(self) -> np.ndarray:
        position, _ = p.getBasePositionAndOrientation(self.body_id)
        return np.asarray(position, dtype=float)
