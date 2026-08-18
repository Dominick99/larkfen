from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SimulationConfig:
    """Values that control physics timing and video output."""

    duration_seconds: float = 6.0
    physics_hz: int = 240
    video_fps: int = 30
    image_width: int = 640
    image_height: int = 480

    def validate(self) -> None:
        if self.duration_seconds <= 0:
            raise ValueError("duration_seconds must be positive")
        if self.physics_hz <= 0 or self.video_fps <= 0:
            raise ValueError("physics_hz and video_fps must be positive")
        if self.physics_hz % self.video_fps != 0:
            raise ValueError("physics_hz must be divisible by video_fps")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("image dimensions must be positive")
