from __future__ import annotations

import numpy as np
import pybullet as p


def render_drone_camera(
    body_id: int, width: int = 640, height: int = 368
) -> tuple[np.ndarray, np.ndarray]:
    """Render a yaw-stabilized, slightly downward-facing pursuit camera."""
    position, orientation = p.getBasePositionAndOrientation(body_id)
    position = np.asarray(position, dtype=float)
    yaw = p.getEulerFromQuaternion(orientation)[2]
    forward = np.array([np.cos(yaw), np.sin(yaw), -0.48])
    camera = position + np.array([0.0, 0.0, -0.04])
    view = p.computeViewMatrix(camera, camera + forward, [0, 0, 1])
    projection = p.computeProjectionMatrixFOV(58, width / height, 0.08, 80)
    _, _, rgba, _, segmentation = p.getCameraImage(
        width,
        height,
        view,
        projection,
        shadow=1,
        renderer=p.ER_TINY_RENDERER,
        flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
    )
    image = np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
    mask = np.asarray(segmentation, dtype=np.int32).reshape(height, width)
    return image, mask


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

