# Drone bounding-box interception

This repository is a training environment for a focused pursuit task: steering
a drone toward and intercepting a moving object using its camera-space bounding
box.

The task begins **after target detection and selection**. Another system may
eventually decide when to activate the pursuit controller, but that system is
outside this repository's scope. Every training episode will start with the
designated moving target already visible in the drone's camera. The pursuit
controller must keep the target in view, infer its relative motion from
bounding-box observations, close the distance, and intercept it.

At a high level, an episode has the following contract:

- **Start:** the moving target is visible, selected, and represented by a valid
  bounding box.
- **Input:** the current bounding box and, where useful, a short history of
  earlier boxes.
- **Output:** steering commands for the drone.
- **Success:** the drone enters a configured interception radius around the
  target.
- **Failure:** the target remains lost, the drone crashes, or the episode times
  out.

The current milestone is the simulation foundation for that task. It includes
a Dockerized, headless PyBullet environment with two airframes: a hovering
quadcopter and an aerodynamic fixed-wing drone. Each currently flies a
repeatable waypoint course suited to its flight characteristics and writes a
video to the host, so no local Python packages are required. Target motion,
bounding-box observations, episode rewards, and training integration will be
added in subsequent milestones.

## Run

Docker Desktop must be running.

```bash
docker compose build simulation
docker compose run --rm simulation
```

The default quadcopter result is written to
`simulation/artifacts/flight.mp4`, along with a JSON file containing the
airframe type, run configuration, and final state.

Select an airframe with `--drone-type`:

```bash
docker compose run --rm simulation \
  python -m drone_sim --drone-type fixed-wing --duration 15 --seed 7 \
  --output /artifacts/fixed-wing.mp4
```

Valid choices are `quadcopter` (the default) and `fixed-wing`. `--mass` can
override the selected airframe's default mass.

## Architecture

- `drone_sim/vehicles/base.py` defines the shared airframe interface.
- `drone_sim/vehicles/quadcopter.py` contains quadcopter geometry and control.
- `drone_sim/vehicles/fixed_wing.py` contains fixed-wing geometry, aerodynamic
  forces, and waypoint autopilot.
- `drone_sim/scene.py` builds the environment.
- `drone_sim/rendering.py` owns camera rendering.
- `drone_sim/runner.py` owns the simulation lifecycle and output metadata.
- `drone_sim/cli.py` parses command-line options and selects an airframe.
- `drone_sim/simulation.py` preserves the original quadcopter API for existing
  callers.

## Current scope

- PyBullet physics in `DIRECT` (headless) mode
- Rigid-body quadcopter and fixed-wing airframes with independent configuration
- Position/attitude control for the quadcopter
- Simplified lift, drag, thrust, banking, and altitude control for fixed-wing
  flight
- Ground, obstacles, a target, shadows, and a tracking camera
- Deterministic seeds and container-friendly MP4 output

PX4, sensor emulation, a Gymnasium API, perception, and domain randomization
are intentionally left for later milestones.
