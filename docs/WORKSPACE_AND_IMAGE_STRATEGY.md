# Peregrine Workspace and Image Strategy

This repo currently has two layers:

1. `docker/`: infrastructure (images, compose, entrypoints, runtime config)
2. `src/`: ROS 2 code (`peregrine_core`, app packages, optional forks like `px4_msgs`)

## How the current Docker stack works

1. `docker-compose.simulation.yml` builds/runs `sim` on x86+NVIDIA with Gazebo + PX4 SITL.
2. `docker-compose.jetson.yml` builds/runs `aircraft` for Orin (arm64, CUDA/TensorRT).
3. `docker-compose.rpi5.yml` builds/runs `aircraft` for RPi5 (arm64, lightweight/headless).
4. All target compose files mount the full repo workspace (`../../`) to `${ROS_WS}`.

Common behavior:

1. `entrypoint.sh` sources ROS + PX4 message workspace + `${ROS_WS}` (default `/ros2_ws`).
2. `DRONE_ID` is mapped to `ROS_DOMAIN_ID`.
3. `ROS_LOCALHOST_ONLY` defaults to `1`.
4. Jetson/RPi set `START_XRCE_AGENT=true` so MicroXRCEAgent starts automatically on default UDP settings.
5. Containers run as non-root user mapped to host UID/GID for bind-mount ownership.

## Recommended source layout

Use `src/` as the single ROS workspace source root:

```text
src/
├── peregrine_core/     # base flight stack (core packages)
├── peregrine_app_<a>/  # app repo A (submodule)
├── peregrine_app_<b>/  # app repo B (submodule)
└── px4_msgs/           # optional local fork/mirror
```

App repos should depend on `peregrine_core` interfaces/libs, not the reverse.

Use Git submodules for reproducible checkout:

```bash
git clone --recurse-submodules <super-repo-url>
cd peregrine
```

Add an app repo:

```bash
git submodule add <app-repo-url> src/peregrine_app_<name>
git commit -m "Add app submodule: peregrine_app_<name>"
```

## Current image naming

For now, builds are local-only and intentionally simple:

1. `ros2-px4-flight-simulation:latest`
2. `ros2-px4-flight-jetson:latest`
3. `ros2-px4-flight-rpi5:latest`

These names are derived from `PROJECT_NAME` in `docker/.env`.

## VS Code workflow

Use the repo root in VS Code and `Dev Containers: Reopen in Container`.

The provided `.devcontainer/devcontainer.json` attaches to the existing `sim` compose service and reuses the same full-workspace bind mount.

Build/test inside container:

```bash
cd ${ROS_WS:-/ros2_ws}
colcon build --symlink-install
source install/setup.bash
```

## Day-to-day commands

Run from `docker/`:

```bash
make build-sim
make sim
make dev
make build-jetson
DRONE_ID=1 make jetson
make build-rpi5
DRONE_ID=1 make rpi5
```
