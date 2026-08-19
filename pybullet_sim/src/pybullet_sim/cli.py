from __future__ import annotations

import argparse
from pathlib import Path

from .config import SimulationConfig
from .runner import run_simulation


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the shared quadcopter in PyBullet and record it."
    )
    parser.add_argument("--duration", type=float, default=6.0, help="Seconds to simulate")
    parser.add_argument("--fps", type=int, default=30, help="Video frames per second")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/simulation.mp4"),
        help="Destination MP4 path",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = SimulationConfig(
        duration_seconds=args.duration,
        video_fps=args.fps,
    )
    output = run_simulation(config, args.output)
    print(f"Video written to {output}")
