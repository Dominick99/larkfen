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


def create_cube() -> int:
    """Create a dynamic cube two metres above the ground."""
    half_extents = [0.35, 0.35, 0.35]
    collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
    )
    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=[0.15, 0.45, 0.9, 1.0],
    )
    return p.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[0.0, 0.0, 2.0],
    )


def create_scene() -> tuple[int, int]:
    """Create every object in the initial learning scene."""
    return create_ground(), create_cube()
