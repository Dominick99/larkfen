from __future__ import annotations

from .contracts import VelocityCommand


def quadcopter_demo_command(elapsed_seconds: float) -> VelocityCommand:
    """Shared command sequence used to compare simulation backends."""
    if elapsed_seconds < 1.0:
        return VelocityCommand()
    if elapsed_seconds < 4.0:
        return VelocityCommand(forward_mps=0.8)
    return VelocityCommand(forward_mps=0.45, yaw_rate_rad_s=0.45)
