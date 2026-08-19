from __future__ import annotations

import argparse
from pathlib import Path

from .config import SimulationConfig
from .runner import run_simulation


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the shared quadcopter model in headless MuJoCo."
    )
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/simulation.mp4"),
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output = run_simulation(
        SimulationConfig(duration_seconds=args.duration, video_fps=args.fps),
        args.output,
    )
    print(f"Video written to {output}")
