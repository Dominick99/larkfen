# Larkfen agent guide

Larkfen is a deterministic PyBullet environment for steering a drone into a
moving target from camera-space bounding boxes. Keep the simulation cheap,
headless, reproducible, and suitable for eventual separation into simulation,
companion-computer guidance, and flight-controller components.

## Start here

- Read `README.md` for the product goal and user-facing commands.
- Use `rg --files` for discovery and `rg` for code search.
- Inspect `git status -sb` before editing; preserve unrelated user changes.
- Generated videos and telemetry belong in `simulation/artifacts/` and must not
  be committed.

## Repository map

- `simulation/drone_sim/runner.py`: simulation lifecycle and telemetry.
- `simulation/drone_sim/perception.py`: segmentation-to-bounding-box adapter.
- `simulation/drone_sim/pursuit.py`: current rule-based guidance policy.
- `simulation/drone_sim/vehicles/`: airframe geometry and low-level control.
- `simulation/drone_sim/rendering.py`: drone and tracking cameras.
- `simulation/drone_sim/target.py`: moving target dynamics.
- `simulation/drone_sim/scene.py`: terrain and obstacles.
- `simulation/drone_sim/cli.py`: command-line interface.
- `docker-compose.yml` and `simulation/Dockerfile`: supported runtime.

## Behavioral invariants

- Guidance may use the bounding box and drone state, but never the target's
  world position.
- Ground-truth target state may be used only for synthetic perception,
  evaluation, and collision detection.
- Preserve deterministic behavior for a fixed seed.
- Keep rendering compatible with PyBullet `DIRECT` and Tiny Renderer.
- Preserve physical-contact telemetry: successful interception requires a
  PyBullet contact, not only proximity.
- Keep tunable values in configuration or clearly named controller constants.

## Verification

For guidance, perception, physics, or controller changes, use the
`verify-interception` skill or run:

```text
python .agents/skills/verify-interception/scripts/verify.py
```

For lightweight documentation-only changes, run `git diff --check`.

## Local skills

- `change-drone-guidance`: use when modifying bounding-box steering,
  interception behavior, or target-loss behavior.
- `verify-interception`: use after changes that can affect runtime behavior.

Keep this file short and update it when component ownership or supported
verification commands change. Put task-specific procedures in `.agents/skills`
instead of expanding this file.
