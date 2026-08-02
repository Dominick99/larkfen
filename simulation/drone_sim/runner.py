from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import imageio.v2 as imageio
import numpy as np
import pybullet as p

from .perception import detect_body, draw_bounding_box
from .pursuit import BoundingBoxPursuitController
from .rendering import render_drone_camera
from .scene import create_scene
from .target import MovingCar
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
            car = MovingCar()
            car.create()
            controller = BoundingBoxPursuitController()

            output.parent.mkdir(parents=True, exist_ok=True)
            writer = imageio.get_writer(
                output, fps=self.config.video_fps, codec="libx264", quality=7
            )
            frame_interval = self.config.physics_hz // self.config.video_fps
            box = None
            detections = 0
            observation_count = 0
            minimum_distance = float("inf")
            last_box = None
            impact = False
            impact_time = None
            impact_speed = None
            for step in range(int(self.config.duration * self.config.physics_hz)):
                elapsed = step / self.config.physics_hz
                car.update(elapsed)
                waypoint = controller.waypoint(drone.body_id, box)
                drone.control(waypoint, 1 / self.config.physics_hz)
                p.stepSimulation()
                position, _, _, _ = drone.state()
                minimum_distance = min(
                    minimum_distance, float(np.linalg.norm(position - car.position()))
                )
                contacts = p.getContactPoints(drone.body_id, car.body_id)
                if contacts:
                    drone_velocity = drone.state()[2]
                    car_velocity = np.asarray(p.getBaseVelocity(car.body_id)[0])
                    impact = True
                    impact_time = (step + 1) / self.config.physics_hz
                    impact_speed = float(np.linalg.norm(drone_velocity - car_velocity))
                    image, mask = render_drone_camera(drone.body_id)
                    box = detect_body(mask, car.body_id)
                    writer.append_data(draw_bounding_box(image, box))
                    break
                if step % frame_interval == 0:
                    image, mask = render_drone_camera(drone.body_id)
                    box = detect_body(mask, car.body_id)
                    observation_count += 1
                    if box is not None:
                        detections += 1
                        last_box = box.as_dict()
                    writer.append_data(draw_bounding_box(image, box))

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
            "target": "moving_car",
            "controller": "bounding_box_intercept",
            "detection_rate": detections / max(observation_count, 1),
            "minimum_target_distance": minimum_distance,
            "last_bounding_box": last_box,
            "impact": impact,
            "impact_time_seconds": impact_time,
            "impact_relative_speed": impact_speed,
        }
        output.with_suffix(".json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
        return metadata

