from __future__ import annotations

import argparse
import json
from pathlib import Path

from .runner import SimulationConfig, SimulationRunner
from .vehicles import FixedWing, FixedWingConfig, Quadcopter, QuadcopterConfig


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a headless PyBullet drone simulation."
    )
    parser.add_argument(
        "--drone-type",
        choices=("quadcopter", "fixed-wing"),
        default="quadcopter",
        help="Airframe and flight model to simulate.",
    )
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output", type=Path, default=Path("/artifacts/flight.mp4"))
    parser.add_argument(
        "--mass",
        type=float,
        default=None,
        help="Override the selected airframe's default mass in kilograms.",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.mass is not None and args.mass <= 0:
        parser.error("--mass must be positive")

    if args.drone_type == "fixed-wing":
        config = FixedWingConfig(
            mass=args.mass if args.mass is not None else FixedWingConfig.mass
        )
        drone = FixedWing(config)
    else:
        config = QuadcopterConfig(
            mass=args.mass if args.mass is not None else QuadcopterConfig.mass
        )
        drone = Quadcopter(config)

    simulation = SimulationConfig(duration=args.duration, seed=args.seed)
    metadata = SimulationRunner(simulation).run(drone, args.output)
    print(json.dumps(metadata, indent=2))
    print(f"Video written to {args.output}")

