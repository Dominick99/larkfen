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

## Standalone food delivery experiment

[`food_delivery/`](food_delivery/README.md) is an independent JSBSim fixed-wing
prototype for geographic navigation between Albuquerque and Santa Fe. It includes
noisy GPS, air-data dead reckoning during outages, automated evaluation, a 2D MP4,
and an interactive map replay. It does not depend on the quadcopter packages.

## Visual car following

[`car_following/`](car_following/README.md) uses MuJoCo to simulate three moving
colored cars and a quadcopter that follows the green car using RGB detection and
tracking boxes. It includes a replaceable perception interface, target-loss
hover behavior, independent tracking evaluation, and a split-view MP4 recording.
