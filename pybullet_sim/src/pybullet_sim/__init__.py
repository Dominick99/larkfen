"""A small, headless PyBullet simulation for learning and experimentation."""

from .config import SimulationConfig
from .runner import run_simulation

__all__ = ["SimulationConfig", "run_simulation"]
