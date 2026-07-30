from __future__ import annotations

import numpy as np
import pybullet as p


def render_tracking_camera(
    body_id: int, width: int = 640, height: int = 368
) -> np.ndarray:
    position, _ = p.getBasePositionAndOrientation(body_id)
    position = np.asarray(position)
    camera = position + np.array([-5.2, -5.2, 3.2])
    view = p.computeViewMatrix(camera, position + [0, 0, 0.25], [0, 0, 1])
    projection = p.computeProjectionMatrixFOV(58, width / height, 0.1, 100)
    _, _, rgba, _, _ = p.getCameraImage(
        width,
        height,
        view,
        projection,
        shadow=1,
        renderer=p.ER_TINY_RENDERER,
    )
    return np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]

