from __future__ import annotations

import numpy as np
import pybullet as p


def render_frame(width: int, height: int) -> np.ndarray:
    """Render the scene with PyBullet's headless Tiny Renderer."""
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[4.0, -5.0, 3.2],
        cameraTargetPosition=[0.0, 0.0, 0.7],
        cameraUpVector=[0.0, 0.0, 1.0],
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=55.0,
        aspect=width / height,
        nearVal=0.1,
        farVal=50.0,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER,
        shadow=1,
    )
    return np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
