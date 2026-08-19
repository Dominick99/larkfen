# PyBullet backend

This directory runs the shared quadcopter and flight controller from
`robotics/` in headless PyBullet. It contains only the PyBullet-specific world,
robot adapter, rendering, and simulation loop.

## Run with Docker

From the repository root:

```bash
docker build -f pybullet_sim/Dockerfile -t larkfen-pybullet-sim .
docker run --rm -v "${PWD}/pybullet_sim/artifacts:/artifacts" larkfen-pybullet-sim
```

On PowerShell, `${PWD}` expands to the current directory. The resulting video
is written to `artifacts/simulation.mp4`.

Optional arguments can be placed after the image name:

```bash
docker run --rm -v "${PWD}/pybullet_sim/artifacts:/artifacts" larkfen-pybullet-sim \
  --duration 5 --fps 30 --output /artifacts/short.mp4
```

## Quadcopter design

The demo uses a body-relative `VelocityCommand`: forward, right, and upward
speed plus yaw rate. A simulator-neutral controller converts that command into
four normalized rotor commands. A simulator-neutral rotor model applies motor
lag and converts throttle to thrust. The `Quadcopter` robot is the thin adapter
that creates the PyBullet body, reads its state, and applies those forces.

```text
VelocityCommand
  -> SimpleVelocityFlightController
  -> RotorModel
  -> PyBullet Quadcopter adapter
```

This keeps the policy-facing contract and core actuator math reusable. The
MuJoCo backend already consumes these same components without changing them.

## Layout

```text
robotics/src/robotics/
  contracts.py          shared policy/controller types
  robot_models/         portable vehicle and actuator math
  flight_controllers/   portable control logic

pybullet_sim/src/pybullet_sim/
  robots/       PyBullet implementations of shared robot models
  config.py     simulation and camera settings
  scene.py      objects placed in the PyBullet world
  rendering.py  camera setup and RGB frame capture
  runner.py     physics loop and video recording
  cli.py        command-line interface
```

The equivalent MuJoCo backend consumes the same shared model, controller, and
commands without importing anything from this directory.
