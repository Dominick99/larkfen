from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import imageio.v2 as imageio
import numpy as np
import pybullet as p

from .rendering import render_tracking_camera
from .scene import create_scene
from .vehicles import Drone


@dataclass(frozen=True)
class SimulationConfig:
    duration: float = 12.0
    seed: int = 1
    physics_hz: int = 240
    video_fps: int = 30

    def validate(self) -> None:
        if self.duration <= 0:
            raise ValueError("duration must be positive")
        if self.physics_hz <= 0 or self.video_fps <= 0:
            raise ValueError("physics_hz and video_fps must be positive")
        if self.physics_hz % self.video_fps:
            raise ValueError("physics_hz must be divisible by video_fps")


class SimulationRunner:
    def __init__(self, config: SimulationConfig) -> None:
        config.validate()
        self.config = config

    def run(self, drone: Drone, output: Path) -> dict[str, Any]:
        np.random.seed(self.config.seed)
        client = p.connect(p.DIRECT)
        if client < 0:
            raise RuntimeError("Could not start PyBullet")

        writer = None
        final_position: tuple[float, ...] = ()
        final_orientation: tuple[float, ...] = ()
        try:
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(1 / self.config.physics_hz)
            p.setPhysicsEngineParameter(
                numSolverIterations=80, deterministicOverlappingPairs=1
            )
            create_scene()
            drone.create()
            waypoints = drone.default_waypoints()

            output.parent.mkdir(parents=True, exist_ok=True)
            writer = imageio.get_writer(
                output, fps=self.config.video_fps, codec="libx264", quality=7
            )
            waypoint_index = 0
            frame_interval = self.config.physics_hz // self.config.video_fps
            for step in range(int(self.config.duration * self.config.physics_hz)):
                position, _, _, _ = drone.state()
                if (
                    np.linalg.norm(waypoints[waypoint_index] - position)
                    < drone.waypoint_radius
                ):
                    waypoint_index = (waypoint_index + 1) % len(waypoints)
                drone.control(
                    waypoints[waypoint_index], 1 / self.config.physics_hz
                )
                p.stepSimulation()
                if step % frame_interval == 0:
                    writer.append_data(render_tracking_camera(drone.body_id))

            final_position, final_orientation = p.getBasePositionAndOrientation(
                drone.body_id
            )
        finally:
            if writer is not None:
                writer.close()
            p.disconnect(client)

        metadata = {
            "drone_type": drone.name,
            "duration_seconds": self.config.duration,
            "seed": self.config.seed,
            "physics_hz": self.config.physics_hz,
            "video_fps": self.config.video_fps,
            "config": drone.config_dict(),
            "final_position": final_position,
            "final_orientation_xyzw": final_orientation,
        }
        output.with_suffix(".json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
        return metadata

