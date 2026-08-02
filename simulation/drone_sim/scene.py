from __future__ import annotations

import pybullet as p
import pybullet_data


def create_scene() -> None:
    """Populate the active physics world with terrain and landmarks."""
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane, -1, rgbaColor=[0.23, 0.31, 0.22, 1])

    for position, size, color in [
        ([2.2, 1.2, 0.55], [0.35, 0.35, 0.55], [0.55, 0.3, 0.16, 1]),
        ([-1.7, 1.8, 0.8], [0.25, 0.25, 0.8], [0.22, 0.42, 0.55, 1]),
        ([0.2, -2.0, 0.4], [0.6, 0.25, 0.4], [0.5, 0.47, 0.2, 1]),
    ]:
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
        p.createMultiBody(0, collision, visual, position)

