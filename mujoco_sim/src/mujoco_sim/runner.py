from __future__ import annotations

import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco

from robotics import VelocityCommand
from robotics.demo import quadcopter_demo_command
from robotics.flight_controllers import SimpleVelocityFlightController

from .config import SimulationConfig
from .rendering import SceneRenderer
from .robots import Quadcopter


def run_simulation(config: SimulationConfig, output: Path) -> Path:
    config.validate()
    output.parent.mkdir(parents=True, exist_ok=True)

    quadcopter, model, data = Quadcopter.create(config.physics_hz)
    controller = SimpleVelocityFlightController(quadcopter.config)
    controller.reset()
    controlled_throttle = controller.update(
        quadcopter.state(), VelocityCommand(), 1.0 / config.control_hz
    )
    renderer = SceneRenderer(model, config.image_width, config.image_height)
    writer = imageio.get_writer(
        output,
        fps=config.video_fps,
        codec="libx264",
        quality=7,
    )
    steps_per_frame = config.physics_hz // config.video_fps
    steps_per_control = config.physics_hz // config.control_hz
    total_steps = round(config.duration_seconds * config.physics_hz)

    try:
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
            mujoco.mj_step(model, data)
            if step % steps_per_frame == 0:
                writer.append_data(renderer.render(data))

        final_state = quadcopter.state()
        metadata = {
            "backend": "mujoco",
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
        writer.close()
        renderer.close()

    return output
