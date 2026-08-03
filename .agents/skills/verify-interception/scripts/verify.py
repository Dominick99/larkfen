from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
ARTIFACT = ROOT / "simulation" / "artifacts" / "agent-verification.json"


def run(*command: str) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def main() -> int:
    run("docker", "compose", "build", "simulation")
    run(
        "docker",
        "compose",
        "run",
        "--rm",
        "simulation",
        "python",
        "-m",
        "drone_sim",
        "--duration",
        "20",
        "--seed",
        "7",
        "--output",
        "/artifacts/agent-verification.mp4",
    )
    telemetry = json.loads(ARTIFACT.read_text(encoding="utf-8"))
    failures: list[str] = []
    if telemetry.get("impact") is not True:
        failures.append("expected physical impact")
    if telemetry.get("detection_rate", 0.0) < 0.95:
        failures.append("detection rate below 0.95")
    if not (telemetry.get("impact_time_seconds") or 0.0) > 0:
        failures.append("missing positive impact time")
    if not (telemetry.get("impact_relative_speed") or 0.0) > 0:
        failures.append("missing positive relative impact speed")
    run("git", "diff", "--check")
    print(json.dumps(telemetry, indent=2))
    if failures:
        print("Verification failed: " + "; ".join(failures), file=sys.stderr)
        return 1
    print(f"Verification passed. Artifacts: {ARTIFACT.with_suffix('.mp4')}, {ARTIFACT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
