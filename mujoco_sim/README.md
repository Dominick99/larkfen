# MuJoCo quadcopter simulation

This backend runs the same shared quadcopter model, rotor dynamics, velocity
controller, and demonstration commands as `pybullet_sim`. Only the world,
robot adapter, state extraction, force application, and rendering use MuJoCo.

## Run with Docker

From the repository root:

```bash
docker build -f mujoco_sim/Dockerfile -t larkfen-mujoco-sim .
docker run --rm -v "${PWD}/mujoco_sim/artifacts:/artifacts" larkfen-mujoco-sim
```

The run writes `simulation.mp4` and `simulation.json` into `artifacts/`.
