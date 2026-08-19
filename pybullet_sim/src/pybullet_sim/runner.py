from __future__ import annotations

import json
from pathlib import Path

import imageio.v2 as imageio
import pybullet as p

from robotics import VelocityCommand
from robotics.demo import quadcopter_demo_command
from robotics.flight_controllers import SimpleVelocityFlightController

from .config import SimulationConfig
from .rendering import render_frame
from .robots import Quadcopter
from .scene import create_ground, create_target_marker


def run_simulation(config: SimulationConfig, output: Path) -> Path:
    """Run the physics loop and record frames to an MP4 file."""
    config.validate()
    output.parent.mkdir(parents=True, exist_ok=True)

    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("PyBullet could not start in DIRECT mode")

    writer = None
    try:
        p.resetSimulation()
        p.setGravity(0.0, 0.0, -9.81)
        p.setTimeStep(1.0 / config.physics_hz)
        create_ground()
        create_target_marker((3.0, 0.0, 0.18))
        quadcopter = Quadcopter()
        quadcopter.create()
        controller = SimpleVelocityFlightController(quadcopter.config)
        controller.reset()
        controlled_throttle = controller.update(
            quadcopter.state(), VelocityCommand(), 1.0 / config.control_hz
        )

        writer = imageio.get_writer(
            output,
            fps=config.video_fps,
            codec="libx264",
            quality=7,
        )
        steps_per_frame = config.physics_hz // config.video_fps
        steps_per_control = config.physics_hz // config.control_hz
        total_steps = round(config.duration_seconds * config.physics_hz)

        for step in range(total_steps):
            elapsed_seconds = step / config.physics_hz
            if step % steps_per_control == 0:
                controlled_throttle = controller.update(
                    quadcopter.state(),
                    quadcopter_demo_command(elapsed_seconds),
                    1.0 / config.control_hz,
                )
            quadcopter.apply_motor_command(
                controlled_throttle,
                1.0 / config.physics_hz,
            )
            p.stepSimulation()
            if step % steps_per_frame == 0:
                writer.append_data(
                    render_frame(config.image_width, config.image_height)
                )

        final_state = quadcopter.state()
        metadata = {
            "backend": "pybullet",
            "scenario": "quadcopter",
            "duration_seconds": config.duration_seconds,
            "physics_hz": config.physics_hz,
            "control_hz": config.control_hz,
            "video_fps": config.video_fps,
            "final_position": final_state.position.tolist(),
            "final_linear_velocity": final_state.linear_velocity.tolist(),
        }
        output.with_suffix(".json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
    finally:
        if writer is not None:
            writer.close()
        p.disconnect(client_id)

    return output
