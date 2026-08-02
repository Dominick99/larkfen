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

The current milestone includes an end-to-end deterministic pursuit baseline.
A car follows a looping course, a forward camera on the drone produces a
ground-truth bounding box from PyBullet's segmentation image, and the drone
steers onto a collision course using the box center and area. The rendered
video is the drone-camera view with the synthetic detection drawn in green. No
target world position is given to the pursuit controller.

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

The demo is split at the same conceptual boundaries used by a deployed drone:

```text
simulation camera -> BoundingBoxObservation
                         |
                         v
companion-computer guidance -> PositionSetpoint
                                  |
                                  v
flight-controller control -> WrenchCommand
                                  |
                                  v
simulation airframe/physics -> VehicleState
```

- `drone_protocol/` defines the shared, simulator-independent messages.
- `companion-computer/` contains bounding-box guidance and has no PyBullet
  dependency.
- `flight-controller/` contains position/attitude control and has no PyBullet
  dependency.
- `simulation/drone_sim/` owns the world, camera, vehicle geometry, PyBullet
  adapter, rendering, and evaluation.

All three currently execute in one deterministic process. The boundaries are
ordinary typed contracts so later transport adapters can put guidance in an
onboard container and replace the Python flight controller with PX4/ArduPilot
SITL or physical autopilot firmware without changing the guidance policy.

## Current scope

- PyBullet physics in `DIRECT` (headless) mode
- Rigid-body quadcopter and fixed-wing airframes with independent configuration
- Simulator-independent quadcopter and fixed-wing flight-control modules
- PyBullet airframes that contain geometry and physics integration only
- A moving car on a deterministic looping path
- Drone-camera segmentation, synthetic bounding boxes, and video overlays
- A bounding-box-only interception controller with physical-contact telemetry
- Ground, obstacles, shadows, and camera rendering
- Deterministic seeds and container-friendly MP4 output

Inter-process transport, MAVLink, PX4/ArduPilot SITL, learned perception, a
Gymnasium API, and domain randomization are intentionally left for later
milestones.
