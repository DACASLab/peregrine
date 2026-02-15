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
3. Jetson/RPi set `START_XRCE_AGENT=true` so MicroXRCEAgent starts automatically.
4. Containers run as non-root user mapped to host UID/GID for bind-mount ownership.

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

## Recommended image layering for GHCR

Use one repository per image family with target and feature tags.

1. Base target image:
   - `ghcr.io/<org>/peregrine-base:sim`
   - `ghcr.io/<org>/peregrine-base:jetson`
   - `ghcr.io/<org>/peregrine-base:rpi5`
2. Core stack image (adds `peregrine_core`, built binaries):
   - `ghcr.io/<org>/peregrine-core:sim`
   - `ghcr.io/<org>/peregrine-core:jetson`
   - `ghcr.io/<org>/peregrine-core:rpi5`
3. App image (extends core and adds app repos):
   - `ghcr.io/<org>/peregrine-app-<name>:sim`
   - `ghcr.io/<org>/peregrine-app-<name>:jetson`
   - `ghcr.io/<org>/peregrine-app-<name>:rpi5`
4. Vision feature variant (optional):
   - `...:jetson-vision`
   - `...:sim-vision`

Keep `vision` as a tag or dedicated Dockerfile variant so non-vision deployments stay smaller.

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
