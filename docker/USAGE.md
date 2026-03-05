# Peregrine Docker Usage

All commands run from the `docker/` directory.

```
cd docker
```

## Prerequisites

- Docker with Compose V2
- NVIDIA drivers + `nvidia-container-toolkit` (simulation and Jetson only)
- X11 display server (for Gazebo and RViz)
- Python 3 (for config generators)

## Configuration

`docker/.env` holds all version pins and defaults. To override values per-machine without editing `.env`, create `docker/.env.local`:

```bash
# Example .env.local on a hardware aircraft
DRONE_ID=2
ZENOH_PORT=7447
```

`.env.local` is gitignored and takes priority over `.env`.

---

## 1. Single-Agent Simulation

One UAV in Gazebo on your dev PC. Good for testing changes to the flight stack.

### Build

```
make build-sim
```

Builds the simulation image (~15-20 GB). Includes CUDA, Gazebo, PX4 SITL, ROS 2, Micro-XRCE-DDS agent, and the Zenoh bridge.

### Run

```
make sim
```

Starts one container (`sim`) with Gazebo + PX4 SITL. Opens an interactive bash shell. From there you can launch the flight stack manually:

```bash
ros2 launch peregrine_bringup single_uav.launch.py
```

### Dev shell

```
make dev
```

Spins up a fresh throwaway container with the workspace mounted. Use this for building, testing, or debugging without touching a running sim.

### Shell into running container

```
make shell-sim
```

Starts the sim container in the background (if not already running) and drops you into a bash shell inside it.

### Teardown

```
make down
```

---

## 2. Multi-Agent Simulation

N UAVs in a shared Gazebo world, each with its own ROS domain and Zenoh bridge. A separate GCS container aggregates telemetry via Zenoh into ROS domain 99.

### Build

```
make build-sim
make build-gcs
```

`build-sim` is used for the UAV containers (they share the simulation image). `build-gcs` builds a lightweight image with RViz and the Zenoh bridge (~3-4 GB, no CUDA/Gazebo/PX4).

### Configure

Edit `docker/.env`:

```
NUM_UAVS=3
```

This controls how many UAVs are spawned and how the GCS Zenoh bridge is wired.

### Run

Terminal 1 -- Gazebo + UAVs:

```
make multi-sitl
```

This auto-generates `compose/docker-compose.multi-sitl.yml` (via `generate_multi_sitl.py`), then brings up:

- `sim` -- shared Gazebo world with N PX4 SITL instances
- `uav1` .. `uavN` -- each runs the Peregrine flight stack in its own ROS domain (1..N) with a Zenoh bridge on ports 7447..7447+N-1

Terminal 2 -- Ground Control Station:

```
make gcs
```

This auto-generates GCS configs (via `generate_gcs_config.py`), then brings up:

- `gcs` -- runs in ROS domain 99, starts a Zenoh bridge that connects to all UAV bridges, then launches tmuxinator with a TUI pane per UAV, an RViz window, and a topics pane

### Interact with GCS

Attach to the GCS tmux session (from the host):

```
make tmux-gcs
```

Or get a bash shell inside the running GCS container:

```
make shell-gcs
```

### Teardown

Stop just the multi-sim UAVs:

```
make multi-sitl-down
```

Stop just the GCS:

```
make gcs-down
```

Stop everything:

```
make down
```

---

## 3. Single-Agent Hardware

One physical aircraft with a companion computer (Jetson or RPi5) connected to a PX4 flight controller over UART.

### Build (on the companion computer)

Jetson Orin:

```
make build-jetson
```

Raspberry Pi 5:

```
make build-rpi5
```

### Configure

Create `docker/.env.local` on the companion computer:

```bash
DRONE_ID=1
ZENOH_PORT=7447
# Override UART device if needed (defaults: Jetson=/dev/ttyTHS1, RPi5=/dev/ttyAMA0)
# XRCE_DEVICE=/dev/ttyUSB0
```

### Run manually

Jetson:

```
make jetson
```

RPi5:

```
make rpi5
```

The container starts the Micro-XRCE-DDS agent on the UART, launches the Zenoh bridge, then runs the flight stack via `tmuxinator start flight`.

### Run on boot (systemd)

Install the systemd service (run once):

```
make install-service PLATFORM=jetson
# or
make install-service PLATFORM=rpi5
```

This creates `/etc/systemd/system/peregrine.service` configured for your platform and working directory.

Enable auto-start:

```
make enable-flight
```

Now the aircraft container starts on every boot. To disable:

```
make disable-flight
```

To remove the service entirely:

```
make uninstall-service
```

### Dev shell

```
make dev-jetson
# or
make dev-rpi5
```

Throwaway container for building or debugging on the companion computer.

---

## 4. Multi-Agent Hardware

Multiple physical aircraft, each running its own companion computer, with a GCS laptop monitoring all of them over the network.

### Aircraft setup (each companion computer)

Follow the single-agent hardware steps above for each aircraft. Give each a unique `DRONE_ID` in its `.env.local`:

```bash
# Aircraft 1 (.env.local)
DRONE_ID=1
ZENOH_PORT=7447

# Aircraft 2 (.env.local)
DRONE_ID=2
ZENOH_PORT=7447
```

Each aircraft's Zenoh bridge listens on port 7447 and publishes telemetry from its ROS domain.

### GCS setup (laptop)

Build the GCS image on your laptop:

```
make build-gcs
```

Create `docker/.env.local` on the GCS laptop:

```bash
GCS_MODE=hardware
GCS_UAV_IPS=192.168.1.10,192.168.1.11
```

`GCS_UAV_IPS` is a comma-separated list of companion computer IPs. The number of IPs determines the number of UAVs -- `GCS_NUM_UAVS` is inferred automatically.

### Run

```
make gcs
```

This generates a Zenoh bridge config that connects to `tcp/<ip>:7447` for each aircraft, generates tmuxinator with a TUI pane per UAV, and starts the GCS container.

Attach to the tmux session:

```
make tmux-gcs
```

---

## What each container runs at startup

Every container uses a shared `entrypoint.sh` that:

1. Sources `/opt/ros/humble/setup.bash` and the workspace overlay
2. Sets `ROS_DOMAIN_ID` from `DRONE_ID`
3. Enables multicast on loopback (needed for DDS + Zenoh co-discovery)
4. Starts the Micro-XRCE-DDS agent if `START_XRCE_AGENT=true` (hardware only)
5. Starts the Zenoh bridge if `ZENOH_BRIDGE_CONFIG` is set (templates `__DRONE_ID__` and `__ZENOH_PORT__` at runtime)
6. Runs the container's `command`

---

## Network architecture

Each UAV runs in its own ROS domain (domain ID = DRONE_ID) with `ROS_LOCALHOST_ONLY=1`. No cross-talk between UAVs over DDS.

Telemetry leaves each UAV via its Zenoh bridge (port 7447+offset in sim, port 7447 on hardware). The GCS Zenoh bridge connects to all UAV bridges and republishes into ROS domain 99, where TUI nodes and RViz subscribe.

```
UAV1 (domain 1) --zenoh:7447--> \
UAV2 (domain 2) --zenoh:7448-->  --> GCS bridge (domain 99) --> TUI + RViz
UAV3 (domain 3) --zenoh:7449--> /
```

---

## Quick reference

| Command | What it does |
|---|---|
| `make build-sim` | Build simulation image |
| `make build-gcs` | Build GCS image |
| `make build-jetson` | Build Jetson image |
| `make build-rpi5` | Build RPi5 image |
| `make build-all` | Build all images |
| `make build-sim NOCACHE=1` | Rebuild from scratch (works with any build target) |
| `make sim` | Single-UAV simulation |
| `make multi-sitl` | Multi-UAV simulation (Gazebo + N UAVs) |
| `make gcs` | Start GCS (auto-detects sim/hardware from GCS_MODE) |
| `make jetson` | Run aircraft on Jetson |
| `make rpi5` | Run aircraft on RPi5 |
| `make dev` | Dev shell (simulation) |
| `make shell-sim` | Shell into running sim container |
| `make shell-gcs` | Shell into running GCS container |
| `make tmux-gcs` | Attach to GCS tmux session |
| `make down` | Stop all containers |
| `make clean` | Stop all + remove images |
| `make nuke` | Stop all + remove images + prune Docker system |
| `make install-service PLATFORM=jetson` | Install systemd auto-start |
| `make enable-flight` | Enable auto-start on boot |
| `make disable-flight` | Disable auto-start |
| `make info` | Print current configuration |
