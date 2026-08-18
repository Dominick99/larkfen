from __future__ import annotations

from pathlib import Path

import imageio.v2 as imageio
import pybullet as p

from .config import SimulationConfig
from .rendering import render_frame
from .scene import create_scene


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
        create_scene()

        writer = imageio.get_writer(
            output,
            fps=config.video_fps,
            codec="libx264",
            quality=7,
        )
        steps_per_frame = config.physics_hz // config.video_fps
        total_steps = round(config.duration_seconds * config.physics_hz)

        for step in range(total_steps):
            p.stepSimulation()
            if step % steps_per_frame == 0:
                writer.append_data(
                    render_frame(config.image_width, config.image_height)
                )
    finally:
        if writer is not None:
            writer.close()
        p.disconnect(client_id)

    return output
