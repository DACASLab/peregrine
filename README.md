# Peregrine Meta Workspace

Workspace for the Peregrine aerial autonomy stack using ROS 2 + PX4 for both simulation and real hardware (Jetson Orin / RPi5 companion setups).

## Repository layout

```text
peregrine/
├── src/
│   ├── peregrine_core/
│   ├── peregrine_app_nav/   # app repo (submodule)
│   ├── peregrine_app_vision/# app repo (submodule)
│   └── px4_msgs/
├── docker/
│   ├── docker/              # Dockerfiles
│   ├── compose/             # compose stacks by target
│   ├── config/              # entrypoint scripts
│   ├── .env                 # version pins
│   └── Makefile             # operational commands
└── .devcontainer/
```

## Quick start

Run commands from `docker/`:

```bash
cd docker
make build-sim
make dev
```

All targets mount the entire repo workspace into `${ROS_WS}` (`ROS_WS` default: `/ros2_ws`), so code and `colcon` artifacts persist on host.
`make dev` opens a disposable shell; use `make sim` / `make jetson` / `make rpi5` for long-running services.
Containers run as a non-root user mapped to host UID/GID (`CONTAINER_USER`, `USER_UID`, `USER_GID` in `docker/.env`).

## Repo management

This workspace is a Git super-repo with submodules under `src/`.
Clone with:

```bash
git clone --recurse-submodules <super-repo-url>
cd peregrine
```

If already cloned without submodules:

```bash
git submodule update --init --recursive
```

Add a new app repo as a submodule:

```bash
git submodule add <app-repo-url> src/peregrine_app_<name>
git commit -m "Add app submodule: peregrine_app_<name>"
```

Update submodules to tracked branches (`.gitmodules`):

```bash
git submodule update --remote --recursive
git add src
git commit -m "Bump submodule pointers"
```

## VS Code

Open this repo root in VS Code, then use `Dev Containers: Reopen in Container`.
The configuration in `.devcontainer/devcontainer.json` reuses the simulation compose service with the same workspace bind mount and non-root container user.

## Architecture and image strategy

See `docs/WORKSPACE_AND_IMAGE_STRATEGY.md` for:

1. Sim vs Jetson vs RPi behavior
2. Core vs app package layout
3. GHCR image layering and suggested tags (including optional `vision` variants)
