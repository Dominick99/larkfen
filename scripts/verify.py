from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PYBULLET_ARTIFACT = ROOT / "pybullet_sim" / "artifacts" / "verification.json"
MUJOCO_ARTIFACT = ROOT / "mujoco_sim" / "artifacts" / "verification.json"


def run(*command: str) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def finite_vector(value: object) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 3
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value)
    )


def main() -> int:
    pybullet_image = "larkfen-pybullet-sim:verification"
    mujoco_image = "larkfen-mujoco-sim:verification"
    run("docker", "build", "-f", "pybullet_sim/Dockerfile", "-t", pybullet_image, ".")
    run("docker", "build", "-f", "mujoco_sim/Dockerfile", "-t", mujoco_image, ".")
    run(
        "docker", "run", "--rm", "--entrypoint", "python", pybullet_image,
        "-m", "unittest", "discover", "-s", "/app/robotics/tests", "-v",
    )
    run(
        "docker", "run", "--rm", "--entrypoint", "python", pybullet_image,
        "-m", "unittest", "discover", "-s", "/app/pybullet_sim/tests", "-v",
    )
    run(
        "docker", "run", "--rm", "--entrypoint", "python", mujoco_image,
        "-m", "unittest", "discover", "-s", "/app/mujoco_sim/tests", "-v",
    )
    run(
        "docker", "run", "--rm", "-v",
        f"{ROOT / 'pybullet_sim' / 'artifacts'}:/artifacts", pybullet_image,
        "--output", "/artifacts/verification.mp4",
    )
    run(
        "docker", "run", "--rm", "-v",
        f"{ROOT / 'mujoco_sim' / 'artifacts'}:/artifacts", mujoco_image,
        "--output", "/artifacts/verification.mp4",
    )

    pybullet = json.loads(PYBULLET_ARTIFACT.read_text(encoding="utf-8"))
    mujoco = json.loads(MUJOCO_ARTIFACT.read_text(encoding="utf-8"))
    failures: list[str] = []
    for backend, telemetry in (("pybullet", pybullet), ("mujoco", mujoco)):
        if not finite_vector(telemetry.get("final_position")):
            failures.append(f"{backend} final position is invalid")
        if not finite_vector(telemetry.get("final_linear_velocity")):
            failures.append(f"{backend} final velocity is invalid")

    if failures:
        position_delta = float("inf")
    else:
        position_delta = math.dist(
            pybullet["final_position"], mujoco["final_position"]
        )
        if position_delta > 0.05:
            failures.append(f"backend position delta {position_delta:.6f} m exceeds 0.05 m")

    run("git", "diff", "--check")
    print(json.dumps({"pybullet": pybullet, "mujoco": mujoco}, indent=2))
    if failures:
        print("Verification failed: " + "; ".join(failures), file=sys.stderr)
        return 1
    print(f"Verification passed. Final position delta: {position_delta:.6f} m")
    print(f"PyBullet artifacts: {PYBULLET_ARTIFACT.with_suffix('.mp4')}, {PYBULLET_ARTIFACT}")
    print(f"MuJoCo artifacts: {MUJOCO_ARTIFACT.with_suffix('.mp4')}, {MUJOCO_ARTIFACT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
