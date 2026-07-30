"""Backward-compatible API for the original quadcopter-only module."""

from pathlib import Path

from .cli import main
from .runner import SimulationConfig, SimulationRunner
from .vehicles import Quadcopter, QuadcopterConfig

DroneConfig = QuadcopterConfig


def run(
    duration: float, seed: int, output: Path, config: DroneConfig
) -> dict:
    """Run a quadcopter simulation using the original function signature."""
    return SimulationRunner(
        SimulationConfig(duration=duration, seed=seed)
    ).run(Quadcopter(config), output)


__all__ = ["DroneConfig", "main", "run"]
