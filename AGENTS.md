# Larkfen agent guide

Larkfen is a simulator-agnostic robotics project. Preserve the boundary between
shared robotics logic and engine-specific adapters.

## Start here

- Read `README.md` for the architecture and Docker commands.
- Use `rg --files` for discovery and inspect `git status -sb` before editing.
- Preserve unrelated user changes.
- Generated recordings and telemetry belong in backend `artifacts/` folders and
  must not be committed.

## Repository map

- `robotics/src/robotics/`: shared contracts, robot models, and controllers.
- `pybullet_sim/`: PyBullet backend, robot adapters, rendering, and Docker image.
- `mujoco_sim/`: MuJoCo backend, robot adapters, rendering, and Docker image.

## Architectural invariants

- Code under `robotics/src/` must not import PyBullet, MuJoCo, Webots, or another
  simulation engine.
- Policies must depend only on canonical contracts, never simulator APIs.
- Simulator robot adapters may create bodies, read state, and apply forces, but
  shared actuator and controller math belongs in `robotics/`.
- Keep units, quaternion ordering, coordinate frames, and timing explicit.
- Preserve deterministic, headless, containerized runs.

## Verification

For robot physics, control, timing, rendering, Docker, or backend changes, run:

```text
python scripts/verify.py
```

For documentation-only changes, run `git diff --check`.
