from __future__ import annotations

import mujoco
import numpy as np


class SceneRenderer:
    def __init__(self, model: mujoco.MjModel, width: int, height: int) -> None:
        self.renderer = mujoco.Renderer(model, height=height, width=width)
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = [1.5, 0.0, 0.7]
        self.camera.distance = 7.0
        self.camera.azimuth = 130.0
        self.camera.elevation = -22.0

    def render(self, data: mujoco.MjData) -> np.ndarray:
        self.renderer.update_scene(data, camera=self.camera)
        return self.renderer.render().copy()

    def close(self) -> None:
        self.renderer.close()
