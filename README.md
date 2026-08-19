# Larkfen

Larkfen is an experimental robotics training project built around one hard
architectural rule: policies, robot dynamics, and flight controllers should not
depend on a particular simulation engine.

The current portability example runs one shared quadcopter model and one shared
velocity flight controller in two independent backends:

```text
                  robotics/
          shared contracts, robot model,
              and flight controller
                    /     \
                   v       v
          pybullet_sim/   mujoco_sim/
```

Both backends run the same hover, forward-flight, and turning command sequence
and write an MP4 recording plus JSON telemetry to their own `artifacts/`
directory.

## Run PyBullet

```bash
docker build -f pybullet_sim/Dockerfile -t larkfen-pybullet-sim .
docker run --rm -v "${PWD}/pybullet_sim/artifacts:/artifacts" larkfen-pybullet-sim
```

## Run MuJoCo

```bash
docker build -f mujoco_sim/Dockerfile -t larkfen-mujoco-sim .
docker run --rm -v "${PWD}/mujoco_sim/artifacts:/artifacts" larkfen-mujoco-sim
```

## Ownership boundaries

- `robotics/` owns simulator-independent contracts, physical parameters,
  actuator math, demonstration commands, and flight-control algorithms.
- `pybullet_sim/` owns only PyBullet world construction, robot instantiation,
  state/force adaptation, rendering, and lifecycle.
- `mujoco_sim/` owns the equivalent MuJoCo-specific implementation.
- AI policies and training will live outside all simulator directories and
  communicate only through the shared contracts.

The two engines are not expected to produce bit-identical results. Cross-backend
tests compare whether they remain within the same useful response envelope.
