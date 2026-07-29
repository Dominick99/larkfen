# Robotics simulation

The first working milestone is a Dockerized, headless PyBullet quadrotor
simulation. It flies a repeatable waypoint course and writes a video to the
host, so no local Python packages are required.

## Run

Docker Desktop must be running.

```bash
docker compose build simulation
docker compose run --rm simulation
```

The result is written to `simulation/artifacts/flight.mp4`, along with a JSON
file containing the run configuration and final state.

To change the run:

```bash
docker compose run --rm simulation \
  python -m drone_sim --duration 15 --seed 7 --output /artifacts/demo.mp4
```

## Current scope

- PyBullet physics in `DIRECT` (headless) mode
- A rigid-body quadrotor with configurable mass and control response
- A basic position/attitude controller following 3D waypoints
- Ground, obstacles, a target, shadows, and a tracking camera
- Deterministic seeds and container-friendly MP4 output

PX4, sensor emulation, a Gymnasium API, perception, and domain randomization
are intentionally left for later milestones.
