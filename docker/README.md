# Peregrine Docker Stack

Multi-platform Docker solution for a ROS 2 / PX4 flight stack across three targets:

| Target | Hardware | Arch | Base Image | GPU | Use Case |
|---|---|---|---|---|---|
| **Simulation** | PC + NVIDIA GPU | amd64 | `nvcr.io/nvidia/cuda:12.8.1` | CUDA + OpenGL | Gazebo Harmonic + PX4 SITL |
| **Jetson** | Orin Nano / NX | arm64 | `nvcr.io/nvidia/l4t-jetpack:r36.4.0` | CUDA + TensorRT | Onboard compute (real drone) |
| **RPi5** | Raspberry Pi 5 | arm64 | `ros:humble-ros-base-jammy` | None | Lightweight companion computer |

Inspired by [aerial-autonomy-stack](https://github.com/JacopoPan/aerial-autonomy-stack) and [aerostack2](https://github.com/aerostack2/aerostack2).

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                .env  (single source of truth)                │
│  ROS_DISTRO · PX4_VERSION · CUDA · base images · etc.      │
└──────────────────────┬──────────────────────────────────────┘
                       │  (docker compose reads .env)
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
  ┌───────────┐  ┌───────────┐  ┌───────────┐
  │ compose/  │  │ compose/  │  │ compose/  │
  │simulation │  │ jetson    │  │ rpi5      │
  │  .yml     │  │  .yml     │  │  .yml     │
  │ build: +  │  │ build: +  │  │ build: +  │
  │ run:      │  │ run:      │  │ run:      │
  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘
        ▼              ▼              ▼
  ┌───────────┐  ┌───────────┐  ┌───────────┐
  │Dockerfile │  │Dockerfile │  │Dockerfile │
  │.simulation│  │.jetson    │  │.rpi5      │
  │           │  │           │  │           │
  │ CUDA base │  │ L4T base  │  │ ROS base  │
  │ ROS2 desk │  │ ROS2 base │  │ ROS2 base │
  │ Gazebo    │  │ CUDA/TRT  │  │           │
  │ PX4 SITL  │  │ GStreamer │  │ Serial    │
  │ px4_msgs  │  │ px4_msgs  │  │ px4_msgs  │
  │ XRCE-DDS  │  │ XRCE-DDS  │  │ XRCE-DDS │
  └───────────┘  └───────────┘  └───────────┘
```

Each compose file handles **both** building and running. All build args
are pulled from `.env` automatically — upgrade any dependency by editing
one file.

---

## Quick Start

### Prerequisites

```bash
# All platforms
sudo apt update && sudo apt install -y docker.io docker-compose-v2 git make

# Simulation PC only: NVIDIA Container Toolkit
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Jetson only: set nvidia as default Docker runtime
# Add to /etc/docker/daemon.json:
#   { "runtimes": {"nvidia": {"path":"nvidia-container-runtime","runtimeArgs":[]}},
#     "default-runtime": "nvidia" }
```

### Build & Run

```bash
git clone <your-repo-url> && cd peregrine/docker

# ── Simulation (PC with NVIDIA GPU) ─────────────────────────
make build-sim            # Build the image
make sim                  # Run it (interactive, with GUI)

# ── Jetson (on the Jetson itself) ────────────────────────────
make build-jetson
DRONE_ID=1 make jetson

# ── RPi5 (on the Pi itself) ─────────────────────────────────
make build-rpi5
DRONE_ID=1 make rpi5
```

### Host UID/GID Mapping (Non-Root User)

Images create a non-root user and map it to host UID/GID so bind-mounted files keep correct ownership on host.

Set these in `docker/.env` before building:

```bash
CONTAINER_USER=peregrine
USER_UID=1000   # id -u
USER_GID=1000   # id -g
```

Tooling notes:

1. `tmuxinator` is installed in all images.
2. Default profile is shipped at `~/.config/tmuxinator/peregrine.yml` inside the container.
3. Start with `tmuxinator start peregrine`.
4. Installations happen as root during image build (normal Docker practice).
5. Runtime config should live under `/home/${CONTAINER_USER}` (for example `~/.config/tmuxinator`), not under `/root`.

Or without the Makefile:

```bash
docker compose -f compose/docker-compose.simulation.yml build
docker compose -f compose/docker-compose.simulation.yml up
```

### Inside the simulation container

```bash
# Terminal 1: start PX4 SITL + Gazebo
cd /opt/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2: start the ROS2 bridge
MicroXRCEAgent udp4 -p 8888

# Terminal 3: verify
ros2 topic list
```

---

## Workspace Mount (Default)

All target compose files mount the host repo root (`../../`) into `${ROS_WS}` (default `${ROS_WS}=/ros2_ws`) so you can edit in VS Code and build inside the container without image rebuilds:
There is no separate dev-override compose file in the current design.

```bash
make dev        # simulation shell
make dev-jetson # jetson shell
make dev-rpi5   # rpi5 shell
```

The `dev*` targets open an interactive shell with the same mounted workspace:

```bash
docker compose -f compose/docker-compose.simulation.yml run --rm sim bash
```

Inside the container:

```bash
cd ${ROS_WS:-/ros2_ws}
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 run your_package your_node
```

---

## Day-to-Day Usage

### `make sim` vs `make dev` vs `make shell-sim`

1. `make sim`
   - Runs the `sim` service with `docker compose up`.
   - Use when you want a long-running simulation container.
   - Best for multi-terminal workflows (PX4, bridge, monitoring in parallel).

2. `make dev`
   - Runs `docker compose run --rm sim bash`.
   - Use for a quick disposable shell (build/test commands, one-off debugging).
   - Container is removed when you exit the shell.

3. `make shell-sim`
   - Runs `docker exec` into an already running `sim` container.
   - Use when `make sim` (or VS Code devcontainer) is already running and you need another terminal.

### VS Code devcontainer behavior

1. `Dev Containers: Reopen in Container` uses `docker-compose.simulation.yml` service `sim`.
2. Workspace opens at `${ROS_WS}` (default `/ros2_ws`) with the same bind mount behavior as CLI.
3. VS Code attaches as the image default non-root user (`CONTAINER_USER`).
4. `remoteUser` is intentionally omitted in `.devcontainer/devcontainer.json` so it follows the Dockerfile `USER` (avoids mismatch if `CONTAINER_USER` changes).
5. Use VS Code terminals for `colcon build`, `ros2 run`, PX4 launch commands.
6. If you changed `ROS_WS`, update `.devcontainer/devcontainer.json` `workspaceFolder` to match.
7. If you also want a normal host terminal into the same container, use `make shell-sim`.

### Recommended flows

1. GUI sim + multiple terminals (CLI):
```bash
cd docker
make sim
# in another host terminal
make shell-sim
```

2. Quick command-only iteration:
```bash
cd docker
make dev
```

3. VS Code-first workflow:
```bash
# Open repo in VS Code -> Reopen in Container
# Then inside integrated terminal:
cd ${ROS_WS:-/ros2_ws}
colcon build --symlink-install
```

4. Hardware shells:
```bash
cd docker
make dev-jetson
make dev-rpi5
```

---

## Adding Your ROS 2 Packages

```bash
# 1. Add an app repo as a submodule (repo root)
cd ..
git submodule add <app-repo-url> src/peregrine_app_<name>
git commit -m "Add app submodule: peregrine_app_<name>"

# 2. Optionally create ROS packages in that app repo
cd src/peregrine_app_<name>
ros2 pkg create --build-type ament_cmake my_app_pkg
git add .
git commit -m "Add my_app_pkg"

# 3. Run dev mode
cd ../../docker
make dev

# 4. Build & test inside the container
cd ${ROS_WS:-/ros2_ws} && colcon build --symlink-install
```

When your package needs a new system or ROS dependency, add the
`apt-get install` line to the relevant Dockerfile(s) and rebuild.

---

## Multi-Drone

Each drone gets a unique `DRONE_ID` (maps to `ROS_DOMAIN_ID`):

```bash
# Terminal 1
DRONE_ID=1 docker compose -f compose/docker-compose.simulation.yml up

# Terminal 2
DRONE_ID=2 docker compose -f compose/docker-compose.simulation.yml up
```

---

## Upgrading

All version pins live in `.env`. Edit and rebuild:

```bash
# Upgrade PX4
# .env → PX4_VERSION=v1.16.0
make build-sim

# Switch ROS 2 distro (Humble → Jazzy)
# .env → ROS_DISTRO=jazzy
# .env → SIM_BASE_IMAGE=nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04
# .env → RPI_BASE_IMAGE=ros:jazzy-ros-base-noble
make build-all

# Upgrade JetPack
# .env → JETSON_BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r37.x.x
make build-jetson
```

---

## Project Structure

```text
peregrine/
├── src/
│   ├── peregrine_core/                # Core flight stack repo
│   ├── peregrine_app_<name>/          # App repo (submodule)
│   └── px4_msgs/                      # Optional local mirror/fork
├── docker/
│   ├── .env                           # ★ Version pins
│   ├── Makefile                       # ★ make build-sim, make dev, etc.
│   ├── docker/
│   │   ├── Dockerfile.simulation
│   │   ├── Dockerfile.jetson
│   │   ├── Dockerfile.rpi5
│   ├── compose/
│   │   ├── docker-compose.simulation.yml
│   │   ├── docker-compose.jetson.yml
│   │   ├── docker-compose.rpi5.yml
│   └── config/
│       └── entrypoint.sh
└── .devcontainer/                     # VS Code entry (recommended)
```

---

## Environment Variables (.env)

| Variable | Default | Where | Description |
|---|---|---|---|
| `ROS_DISTRO` | `humble` | All | ROS 2 distribution |
| `PX4_VERSION` | `v1.15.4` | Sim | PX4 firmware tag for SITL |
| `DRONE_ID` | `1` | All | Sets `ROS_DOMAIN_ID` |
| `PX4_SIM_MODEL` | `x500` | Sim | Gazebo vehicle model |
| `PX4_GZ_WORLD` | `default` | Sim | Gazebo world file |
| `HEADLESS` | `false` | Sim | Run Gazebo headless |
| `ROS_WS` | `/ros2_ws` | All | ROS 2 workspace path inside container |
| `XRCE_DDS_PORT` | `8888` | All | XRCE-DDS Agent UDP port |
| `START_XRCE_AGENT` | `false` | Deploy | Auto-start XRCE-DDS agent on boot |
| `CONTAINER_USER` | `peregrine` | All | Non-root username inside container |
| `USER_UID` | `1000` | All | Host UID to map container user |
| `USER_GID` | `1000` | All | Host GID to map container user |

---

## Make Targets

| Command | Description |
|---|---|
| `make build-sim` | Build simulation image |
| `make build-jetson` | Build Jetson image |
| `make build-rpi5` | Build RPi5 image |
| `make build-all` | Build all images |
| `make sim` | Run simulation (interactive, GUI) |
| `make jetson` | Run Jetson container |
| `make rpi5` | Run RPi5 container |
| `make dev` | Sim interactive shell |
| `make dev-jetson` | Jetson interactive shell |
| `make shell-sim` | Exec bash into running sim container |
| `make down` | Stop all containers |
| `make clean` | Stop + remove project images |

---

## References

- [aerial-autonomy-stack](https://github.com/JacopoPan/aerial-autonomy-stack) — 3-image Docker architecture, Jetson deployment
- [aerostack2](https://github.com/aerostack2/aerostack2) — ROS 2 multi-UAV framework
- [AirStack (CMU)](https://github.com/castacks/AirStack) — Docker-based autonomy boilerplate
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/)
- [dustynv/ros Jetson containers](https://hub.docker.com/r/dustynv/ros)
