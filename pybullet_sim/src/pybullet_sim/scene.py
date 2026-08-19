from __future__ import annotations

import pybullet as p


def create_ground() -> int:
    """Create a large, static ground plane without external asset files."""
    collision_shape = p.createCollisionShape(
        p.GEOM_PLANE,
        planeNormal=[0.0, 0.0, 1.0],
    )
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=-1,
    )


def create_target_marker(position: tuple[float, float, float]) -> int:
    """Create a visible, non-colliding marker for the quadcopter demo."""
    visual_shape = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.16,
        rgbaColor=[0.95, 0.18, 0.12, 1.0],
    )
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
    )
