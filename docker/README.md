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

## Dev Mode

Mount the host `../src` into `/ros2_ws/src` so you can edit in VS Code
and build inside the container without image rebuilds:

```bash
make dev        # simulation with source mounted
make dev-jetson # jetson with source mounted
make dev-rpi5   # rpi5 with source mounted
```

For simulation this uses `docker-compose.dev.yml`:

```bash
docker compose \
  -f compose/docker-compose.simulation.yml \
  -f compose/docker-compose.dev.yml \
  run --rm sim bash
```

For Jetson / RPi5 dev shells, the Makefile uses `docker-compose.dev.aircraft.yml`.

Inside the container:

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 run your_package your_node
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
cd /ros2_ws && colcon build --symlink-install
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

## Cross-Host Communication (Sim PC ↔ Jetson ↔ RPi5)

Edit `config/cyclonedds.xml`, uncomment the `<Peers>` block and add IPs:

```xml
<Peers>
  <Peer address="192.168.1.100"/>  <!-- Sim PC -->
  <Peer address="192.168.1.101"/>  <!-- Jetson -->
  <Peer address="192.168.1.102"/>  <!-- RPi5 -->
</Peers>
```

For wireless / NAT-heavy setups, consider adding the
[Zenoh ROS 2 bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds).

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
│   │   └── Dockerfile.deps
│   ├── compose/
│   │   ├── docker-compose.simulation.yml
│   │   ├── docker-compose.jetson.yml
│   │   ├── docker-compose.rpi5.yml
│   │   ├── docker-compose.dev.yml
│   │   └── docker-compose.dev.aircraft.yml
│   └── config/
│       ├── entrypoint.sh
│       └── cyclonedds.xml
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
| `XRCE_DDS_PORT` | `8888` | All | XRCE-DDS Agent UDP port |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | All | DDS middleware |
| `START_XRCE_AGENT` | `false` | Deploy | Auto-start XRCE-DDS agent on boot |

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
| `make dev` | Sim with host source mounted |
| `make dev-jetson` | Jetson with host source mounted |
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
