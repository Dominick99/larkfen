# Minimal PyBullet simulation

This directory is a self-contained, headless PyBullet example. It does not use
the drone code elsewhere in the repository.

The initial scene contains a ground plane and a cube. The cube starts above the
plane, falls under gravity, and settles on the ground. PyBullet runs in
`DIRECT` mode, and its Tiny Renderer produces frames for an MP4 file.

## Run with Docker

From this directory:

```bash
docker build -t larkfen-pybullet-sim .
docker run --rm -v "${PWD}/artifacts:/artifacts" larkfen-pybullet-sim
```

On PowerShell, `${PWD}` expands to the current directory. The resulting video
is written to `artifacts/simulation.mp4`.

Optional arguments can be placed after the image name:

```bash
docker run --rm -v "${PWD}/artifacts:/artifacts" larkfen-pybullet-sim \
  --duration 5 --fps 30 --output /artifacts/short.mp4
```

## Layout

```text
src/pybullet_sim/
  config.py     simulation and camera settings
  scene.py      objects placed in the PyBullet world
  rendering.py  camera setup and RGB frame capture
  runner.py     physics loop and video recording
  cli.py        command-line interface
```

This separation is deliberately small: each module corresponds to one part of
a basic simulation and can be changed independently while learning PyBullet.
