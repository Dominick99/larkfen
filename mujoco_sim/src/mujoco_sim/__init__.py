"""Headless MuJoCo backend for shared robotics models and controllers."""

from .config import SimulationConfig
from .runner import run_simulation

__all__ = ["SimulationConfig", "run_simulation"]
