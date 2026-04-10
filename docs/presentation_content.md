# PEREGRINE Flight Stack — Project Overview Presentation Content

**Presentation Duration:** 3–4 hours (excluding Q&A)  
**Audience:** Funding agency technical review committee  
**Assumed Background:** General robotics/software; ROS2 and PX4 familiarity varies

---

## Table of Contents

- [Part A: Background and Fundamentals (Backup/Reference)](#part-a-background-and-fundamentals)
- [Part B: Design Details and System Overview](#part-b-design-details-and-system-overview)
- [Part C: Code Overview](#part-c-code-overview)

---

# Part A: Background and Fundamentals

> These slides serve as backup/reference material. Present selectively based on audience familiarity.

---

## A.1 ROS2 Fundamentals

### A.1.1 What is ROS2?

- Robot Operating System 2 — middleware framework for robotics software
- Not an OS — a set of libraries, tools, and conventions for building robot applications
- Built on top of DDS (Data Distribution Service) for real-time, distributed communication
- Language support: C++, Python (PEREGRINE uses C++17)

### A.1.2 Core Communication Primitives

**Topics** — Asynchronous publish/subscribe

```
Publisher ──[message]──> Topic ──> Subscriber(s)

- Many-to-many
- Fire-and-forget (no acknowledgement)
- Used for: sensor data streams, state estimates, control outputs
- Example: estimator publishes State at 250Hz, controller subscribes
```

**Services** — Synchronous request/response

```
Client ──[request]──> Service Server ──[response]──> Client

- One-to-one per call
- Blocking (caller waits for response)
- Used for: configuration changes, one-shot commands
- Example: switching active controller plugin
```

**Actions** — Asynchronous goal/feedback/result

```
Client ──[goal]──> Action Server
         <──[feedback]── (periodic updates)
         <──[result]── (on completion)

- Long-running tasks with progress tracking
- Cancellable
- Used for: takeoff, landing, trajectory execution
- Example: takeoff to 5m, feedback = current altitude, result = success/fail
```

### A.1.3 Quality of Service (QoS)

QoS profiles control communication reliability and behavior:

| Profile | Reliability | Durability | Use Case |
|---------|------------|------------|----------|
| Sensor Data | Best Effort | Volatile | High-rate streams (IMU, state) |
| Parameters | Reliable | Transient Local | Configuration, late-joining nodes |
| Default | Reliable | Volatile | Services, actions |

- **Best Effort**: Drop messages if subscriber can't keep up (preferred for real-time)
- **Reliable**: Retransmit until received (needed for commands)
- **Transient Local**: Late-joining subscribers get last published value
- QoS mismatch between publisher and subscriber = silent communication failure

### A.1.4 Nodes, Executors, and Callback Groups

```
┌─────────────────────────────────────────────┐
│              ROS2 Process                    │
│                                             │
│   ┌─────────────────────────────────────┐   │
│   │       MultiThreadedExecutor         │   │
│   │                                     │   │
│   │   ┌──────────┐    ┌──────────┐     │   │
│   │   │  Node A  │    │  Node B  │     │   │
│   │   │ (hw_abs) │    │ (est_mgr)│     │   │
│   │   └──────────┘    └──────────┘     │   │
│   └─────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
```

- **Node**: Smallest unit of computation — owns subscribers, publishers, services
- **Executor**: Dispatches callbacks from nodes to threads
  - `SingleThreadedExecutor`: One callback at a time
  - `MultiThreadedExecutor`: Concurrent callbacks across callback groups
- **Callback Groups**:
  - `MutuallyExclusive`: Only one callback in the group runs at a time (implicit mutex)
  - `Reentrant`: Multiple callbacks can run concurrently (user manages thread safety)

### A.1.5 Lifecycle (Managed) Nodes

Standard state machine for deterministic startup/shutdown:

```
     ┌──────────────┐
     │  Unconfigured │
     └──────┬───────┘
            │ configure()
            ▼
     ┌──────────────┐
     │   Inactive    │
     └──────┬───────┘
            │ activate()
            ▼
     ┌──────────────┐
     │    Active     │  ← Normal operation
     └──────┬───────┘
            │ deactivate()
            ▼
     ┌──────────────┐
     │   Inactive    │
     └──────────────┘
```

- Nodes declare parameters and create subscriptions in `on_configure()`
- Nodes begin processing in `on_activate()`
- Enables coordinated startup: hardware connects before estimator starts
- PEREGRINE managers use lifecycle nodes with `auto_start` for self-driven transitions

### A.1.6 Namespaces and Multi-Robot

```
# Single UAV
/hardware_abstraction/px4_state
/estimator_manager/state

# Multi-UAV with namespaces
/uav1/hardware_abstraction/px4_state
/uav1/estimator_manager/state
/uav2/hardware_abstraction/px4_state
/uav2/estimator_manager/state
```

- ROS2 namespaces isolate topic/service names per robot
- DDS Domain IDs provide network-level isolation
- Combined with `ROS_LOCALHOST_ONLY=1` for container isolation

### A.1.7 Launch System

- Python-based launch files for composing multi-node systems
- Supports launch arguments, conditionals, environment variables
- Can include other launch files for modularity

```python
# Example: Launch a node with parameters from YAML
Node(
    package='estimator_manager',
    executable='estimator_manager_node',
    namespace='uav1',
    parameters=[config_yaml_path],
)
```

### A.1.8 pluginlib — Runtime Plugin Loading

```
┌────────────────────────────────────┐
│         Manager Node               │
│                                    │
│   ClassLoader<ControllerBase>      │
│        │                           │
│        ├── load("px4_passthrough") │
│        ├── load("se3_controller")  │
│        └── load("custom_ctrl")     │
└────────────────────────────────────┘
```

- Load C++ classes at runtime from shared libraries
- Base class defines interface, plugins implement it
- Registered via XML descriptors + `PLUGINLIB_EXPORT_CLASS` macro
- No recompilation of manager needed to add new algorithms

---

## A.2 PX4 Fundamentals

### A.2.1 PX4 Autopilot Overview

- Open-source autopilot for drones, VTOL, rovers
- Handles low-level flight control: attitude stabilization, motor mixing, failsafes
- Runs on dedicated flight controller hardware (Pixhawk, CUAV, etc.)
- PEREGRINE interfaces with PX4 as a companion computer stack

### A.2.2 PX4 Control Architecture

```
                    PX4 Internal Control Stack
┌─────────────────────────────────────────────────┐
│                                                 │
│   Position Controller                           │
│        │  (position error → velocity cmd)       │
│        ▼                                        │
│   Velocity Controller                           │
│        │  (velocity error → acceleration cmd)   │
│        ▼                                        │
│   Attitude Controller                           │
│        │  (orientation error → rate cmd)         │
│        ▼                                        │
│   Rate Controller                               │
│        │  (angular rate error → torque cmd)      │
│        ▼                                        │
│   Motor Mixer                                   │
│        │  (torque + thrust → individual motors) │
│        ▼                                        │
│   PWM / DShot Output                            │
│                                                 │
└─────────────────────────────────────────────────┘
```

PEREGRINE can inject commands at any level:
- **Position/Velocity mode**: PX4 handles all control layers
- **Attitude mode**: PEREGRINE computes desired orientation + thrust, PX4 handles rate control
- **Body Rate mode**: PEREGRINE computes angular rates + thrust, PX4 handles motor mixing only

### A.2.3 Offboard Mode

- PX4 flight mode that accepts external commands from a companion computer
- Requirements:
  - Must publish `OffboardControlMode` at >= 2 Hz or PX4 exits offboard
  - Must start publishing before requesting mode switch
  - Setpoint stream determines control level (position/velocity/attitude/rate)

### A.2.4 uXRCE-DDS Bridge

```
┌─────────────┐     UDP      ┌─────────────────┐    DDS     ┌──────────┐
│     PX4     │◄────────────►│ MicroXRCE Agent │◄──────────►│   ROS2   │
│  (uORB)    │   (serial/   │                 │            │  Nodes   │
│             │    ethernet) │                 │            │          │
└─────────────┘              └─────────────────┘            └──────────┘
```

- Replaces MAVLink for ROS2 integration
- PX4 publishes/subscribes uORB topics via DDS
- MicroXRCE Agent bridges PX4's micro-DDS client to full DDS network
- Topics appear as `/fmu/out/*` (from PX4) and `/fmu/in/*` (to PX4)

### A.2.5 Frame Conventions

```
PX4 uses NED/FRD:                    ROS2 uses ENU/FLU:
  North (+X)                            East (+X)
     │                                     │
     │                                     │
     └──────► East (+Y)                    └──────► North (+Y)
    ╱                                     ╱
   ▼                                     ▲
  Down (+Z)                             Up (+Z)

Body: Forward/Right/Down              Body: Forward/Left/Up
```

- This mismatch is a major source of bugs in PX4+ROS2 systems
- PEREGRINE enforces all internal computation in ENU/FLU
- Conversion happens at exactly one point: `hardware_abstraction`

### A.2.6 PX4 SITL (Software-In-The-Loop)

- Full PX4 autopilot running on host machine (no hardware needed)
- Connects to Gazebo Harmonic for physics simulation
- Identical autopilot code to what runs on hardware
- Multi-instance support: run N PX4 instances with different ports/domain IDs

---

# Part B: Design Details and System Overview

---

## B.1 Project Overview

### B.1.1 What is PEREGRINE?

**PEREGRINE** (Aerial Robotics Infrastructure for Operational Navigation) is a ROS2-based autonomous flight stack for multi-agent UAV operations with PX4 autopilots.

**Key Characteristics:**
- Manager-based modular architecture
- Plugin-based algorithms (controllers, estimators, trajectory generators)
- Multi-platform deployment: simulation, Jetson Orin, Raspberry Pi 5
- Multi-agent support with collision avoidance
- Safety-first design with layered monitoring
- Containerized (Docker) with per-target images

### B.1.2 Design Goals

| Goal | Approach |
|------|----------|
| **Modularity** | Each functional domain is a separate ROS2 package with clean interfaces |
| **Extensibility** | Plugin architecture — add new algorithms without modifying core |
| **Safety** | Three-layer safety: manager checks, safety monitor, PX4 hardware failsafes |
| **Multi-agent** | Namespace isolation, Buffered Voronoi Cell collision avoidance |
| **Deployability** | Docker images per platform, systemd for hardware deployment |
| **Research-friendly** | Swap controllers/estimators at runtime via service calls |

### B.1.3 Technology Stack

| Layer | Technology |
|-------|-----------|
| Language | C++17 |
| Middleware | ROS2 Humble/Jazzy |
| Autopilot | PX4 v1.15+ via uXRCE-DDS |
| Simulation | Gazebo Harmonic, PX4 SITL |
| Math | Eigen3 |
| Plugins | ROS2 pluginlib |
| Visualization | RViz2, custom ncurses TUI |
| Containerization | Docker, Docker Compose |
| Inter-container comms | Zenoh bridge |
| Hardware targets | Jetson Orin, Raspberry Pi 5 |

---

## B.2 System Architecture

### B.2.1 Layered Architecture

```
Layer 4: Coordination    ┌─────────────────────────────────┐
                         │    multi_agent_coordinator       │
                         │    (planned: BVC, consensus)     │
                         └─────────────────────────────────┘
                                        │
Layer 3: Orchestration   ┌─────────────────────────────────┐
                         │         uav_manager              │
                         │  (state machine, lifecycle mgmt) │
                         └─────────────────────────────────┘
                                        │
                         ┌──────────┬───┴───┬──────────┐
Layer 2: Functional      │          │       │          │
                    ┌────┴────┐ ┌───┴───┐ ┌─┴────┐ ┌───┴────┐
                    │estimator│ │control│ │traj  │ │safety  │
                    │_manager │ │_manager│ │_mgr  │ │_monitor│
                    └────┬────┘ └───┬───┘ └─┬────┘ └───┬────┘
                         │          │       │          │
Layer 1: Infrastructure  └──────────┴───┬───┴──────────┘
                         ┌──────────────┴──────────────┐
                         │     hardware_abstraction     │
                         │      frame_transforms        │
                         └──────────────┬──────────────┘
                                        │
Layer 0: External        ┌──────────────┴──────────────┐
                         │     PX4 (via uXRCE-DDS)      │
                         └─────────────────────────────┘
```

### B.2.2 Package Map

| Package | Type | Layer | Purpose |
|---------|------|-------|---------|
| `peregrine_interfaces` | Interface definitions | Cross-cutting | All custom messages, services, actions |
| `frame_transforms` | Library | 1 — Infrastructure | ENU/NED conversions, TF2 broadcasting |
| `hardware_abstraction` | Node | 1 — Infrastructure | Sole PX4 interface, frame conversion boundary |
| `estimation_manager` | Node + Plugins | 2 — Functional | State estimation plugin management |
| `control_manager` | Node + Plugins | 2 — Functional | Flight controller plugin management |
| `trajectory_manager` | Node + Plugins | 2 — Functional | Trajectory generation and evaluation |
| `safety_monitor` | Node | 2 — Functional | Geofence, heartbeats, envelope protection |
| `uav_manager` | Node | 3 — Orchestration | FSM, lifecycle, takeoff/land sequences |
| `multi_agent_coordinator` | Node | 4 — Coordination | (Planned) BVC collision avoidance, fleet state |
| `tui_status` | Node | Tooling | ncurses real-time monitoring display |
| `rviz_plugins` | Node | Tooling | RViz2 flight visualization |
| `peregrine_bringup` | Launch/Config | Deployment | Launch files, YAML configs, simulation setup |

### B.2.3 Data Flow — Control Loop

```
                              PX4 Autopilot
                                   │
                     /fmu/out/vehicle_odometry (NED/FRD, 250Hz)
                                   │
                                   ▼
                        ┌─────────────────────┐
                        │ hardware_abstraction │
                        │  NED→ENU, FRD→FLU   │
                        └──────────┬──────────┘
                                   │
                    /hardware_abstraction/px4_state (ENU/FLU, 250Hz)
                                   │
                                   ▼
                        ┌─────────────────────┐
                        │  estimation_manager  │
                        │  (active plugin)     │
                        └──────────┬──────────┘
                                   │
               /estimation_manager/state (ENU/FLU, 250Hz)
                                   │
                    ┌──────────────┴──────────────┐
                    │                             │
                    ▼                             ▼
         ┌─────────────────┐          ┌─────────────────────┐
         │ control_manager │◄─────────│  trajectory_manager  │
         │ (active plugin) │ setpoint │  (active plugin)     │
         └────────┬────────┘  (50Hz)  └─────────────────────┘
                  │
     /control_manager/control_output (ENU/FLU, 250Hz)
                  │
                  ▼
         ┌─────────────────────┐
         │ hardware_abstraction │
         │  ENU→NED, FLU→FRD   │
         └──────────┬──────────┘
                    │
         /fmu/in/trajectory_setpoint (NED/FRD)
         /fmu/in/vehicle_attitude_setpoint (NED/FRD)
         /fmu/in/vehicle_rates_setpoint (NED/FRD)
                    │
                    ▼
              PX4 Autopilot
```

### B.2.4 Timing Requirements

| Loop | Frequency | Deadline | Notes |
|------|-----------|----------|-------|
| State estimation | 250 Hz | 4 ms | Matched to PX4 odometry rate |
| Control output | 250 Hz | 4 ms | Must be deterministic |
| Trajectory evaluation | 50 Hz | 20 ms | Setpoints interpolated at controller rate |
| Safety monitoring | 100 Hz | 10 ms | Highest priority — independent watchdog |
| Offboard heartbeat | >= 2 Hz | 500 ms | PX4 requirement — exits offboard if missed |
| TUI display | 10 Hz | 100 ms | Non-critical |

### B.2.5 Node Communication Matrix

| Publisher | Subscriber | Topic | Message Type | Rate |
|-----------|-----------|-------|-------------|------|
| hardware_abstraction | estimation_manager | `/px4_state` | `State` | 250 Hz |
| estimation_manager | control_manager, safety_monitor, uav_manager | `/estimated_state` | `State` | 250 Hz |
| trajectory_manager | control_manager | `/trajectory_setpoint` | `TrajectorySetpoint` | 50 Hz |
| control_manager | hardware_abstraction | `/control_output` | `ControlOutput` | 250 Hz |
| safety_monitor | uav_manager | `/safety_status` | `SafetyStatus` | 10 Hz |
| uav_manager | all managers | `/uav_mode` | `UAVMode` | 10 Hz |

---

## B.3 Frame Convention Strategy

### B.3.1 The Problem

PX4 and ROS2 use different coordinate frame conventions. Mixing them up is the single most common source of "drone flew in the wrong direction" bugs.

### B.3.2 PEREGRINE's Solution: Single Conversion Point

**Golden Rule:** All PEREGRINE packages operate in ENU/FLU. Conversion to/from PX4's NED/FRD happens exclusively in `hardware_abstraction`.

```
┌─────────────────────────────────────────────────────────┐
│                 PEREGRINE STACK                          │
│           Everything in ENU/FLU                         │
│    estimator, controller, trajectory, safety, TUI       │
└─────────────────────────────┬───────────────────────────┘
                              │
                   ┌──────────▼──────────┐
                   │ hardware_abstraction │
                   │    CONVERSION ZONE   │
                   │   ENU ↔ NED          │
                   │   FLU ↔ FRD          │
                   └──────────┬──────────┘
                              │
┌─────────────────────────────▼───────────────────────────┐
│                    PX4 AUTOPILOT                         │
│            Everything in NED/FRD                         │
└─────────────────────────────────────────────────────────┘
```

### B.3.3 Conversion Summary

| Quantity | ENU → NED | FLU → FRD |
|----------|-----------|-----------|
| Position/Velocity | `(x,y,z) → (y, x, -z)` | `(x,y,z) → (x, -y, -z)` |
| Quaternion | `(w,x,y,z) → (w, y, x, -z)` | Same transformation |
| Yaw | `yaw_ned = π/2 - yaw_enu` | N/A |
| Covariance | `C_ned = R · C_enu · R^T` | Same pattern |

### B.3.4 TF2 Frame Tree

```
world (ENU)
├── odom (ENU)
│   └── base_link (FLU)
│       ├── imu_link
│       ├── camera_front
│       ├── gps_link
│       └── lidar_link
│
├── uav1/base_link    (multi-UAV)
├── uav2/base_link
└── uavN/base_link
```

---

## B.4 Plugin Architecture

### B.4.1 Design Rationale

Traditional approach: modify core code to change an algorithm.  
PEREGRINE approach: load a new plugin at runtime.

| Aspect | Without Plugins | With Plugins |
|--------|----------------|--------------|
| Adding new controller | Modify core, recompile, redeploy | Compile plugin library, load via YAML config |
| Runtime switching | Not possible | Service call: `set_controller("se3")` |
| Testing | Full system required | Mock the manager interface |
| Multiple algorithms | Preprocessor guards or refactoring | All loaded simultaneously, switch at will |

### B.4.2 Plugin Pattern (Common to All Managers)

```
Manager Node
    │
    ├── ClassLoader<Base>        ← pluginlib loader
    │       │
    │       ├── Plugin A         ← loaded from shared library
    │       ├── Plugin B
    │       └── Plugin C
    │
    ├── active_plugin_            ← pointer to currently active
    │
    ├── Subscriber (input data)
    ├── Publisher (output data)
    └── Service (set_plugin)     ← runtime switching
```

Each plugin type follows the same lifecycle:
1. `initialize(node, name)` — receive parent node for parameters/logging
2. `activate()` — start processing
3. `deactivate()` — stop processing
4. `reset()` — clear internal state

### B.4.3 Three Plugin Domains

| Domain | Base Class | Input | Output | Current Plugins |
|--------|-----------|-------|--------|-----------------|
| Estimation | `EstimatorBase` | PX4 state, external pose | `State` | PX4 Passthrough |
| Control | `ControllerBase` | State + TrajectorySetpoint | `ControlOutput` | PX4 Passthrough, SE(3) Geometric |
| Trajectory | `TrajectoryGeneratorBase` | Waypoints / goals | `TrajectorySetpoint` stream | Waypoint Linear, Hold Position, Takeoff/Land |

---

## B.5 Safety Architecture

### B.5.1 Three-Layer Safety

```
Layer 3: PX4 Hardware Failsafes (last resort — not controlled by PEREGRINE)
  RC loss, battery failsafe, hardware geofence, motor failure detection

Layer 2: PEREGRINE Safety Monitor (this package)
  Software geofence, heartbeat monitoring, state validation,
  flight envelope protection, battery monitoring, collision alerts

Layer 1: Manager-Level Checks
  Controller output limits, estimator confidence thresholds,
  trajectory constraint validation
```

### B.5.2 Safety Monitor Checks

| Check | Rate | Trigger Level | Action |
|-------|------|--------------|--------|
| Software geofence (cylindrical/polygonal) | 100 Hz | WARNING → EMERGENCY | Warn → Land/RTH |
| Heartbeat monitoring (per-node) | Continuous | CRITICAL / HIGH / MEDIUM | Emergency land → RTH → Warn |
| State validation (NaN, divergence) | 100 Hz | EMERGENCY | Hover → Land |
| Flight envelope (velocity, tilt, altitude) | 100 Hz | CAUTION | Limit output, warn |
| Battery monitoring (3 thresholds) | 10 Hz | WARNING → CRITICAL → EMERGENCY | Warn → RTH → Immediate land |
| Inter-UAV collision distance | 20 Hz | WARNING → EMERGENCY | Slow → Emergency stop |

### B.5.3 Safety Status State Machine

```
  NOMINAL ──violation──► WARNING ──persists──► CAUTION ──critical──► EMERGENCY
     ▲                      │                                          │
     └──────── cleared ─────┘                              manual reset│
     ◄─────────────────────────────────────────────────────────────────┘
```

### B.5.4 Heartbeat Priority System

| Source | Priority | Timeout | Loss Action |
|--------|----------|---------|-------------|
| hardware_abstraction (PX4 link) | CRITICAL | 1.0 s | Emergency land |
| estimation_manager | CRITICAL | 0.5 s | Emergency land |
| control_manager | HIGH | 0.5 s | Switch to backup / RTH |
| GCS connection | HIGH | 5.0 s | RTH |
| trajectory_manager | MEDIUM | 1.0 s | Hold position |

---

## B.6 UAV Manager — State Machine

### B.6.1 State Machine

```
┌────────┐  arm()   ┌────────┐  takeoff()  ┌──────────┐
│  IDLE  │────────►│ ARMED  │────────────►│ TAKING   │
│        │         │        │             │   OFF    │
└───┬────┘         └───┬────┘             └────┬─────┘
    ▲                  │                       │
    │ disarm()         │ disarm()              ▼
    │                  ▼                  ┌──────────┐
┌───┴────┐         ┌────────┐            │ HOVERING │
│ LANDED │◄────────│LANDING │◄───────────┤          │
│        │ touched │        │   land()   └────┬─────┘
└────────┘  down   └────────┘                 │
                        ▲                     ▼
                        │ land()         ┌──────────┐
                        │                │  FLYING  │
                        │ rth()          │          │
                   ┌────┴───┐            └──────────┘
                   │RETURNING│◄───────────────┘
                   │  HOME  │
                   └────────┘

From ANY state: emergency_stop() → EMERGENCY (requires manual recovery)
```

### B.6.2 Preflight Check Sequence

Before arming is permitted:

1. `hardware_abstraction` connected to PX4 (heartbeat alive)
2. `estimation_manager` healthy and publishing state
3. `control_manager` has active controller loaded
4. `safety_monitor` reports NOMINAL (no active warnings/errors)
5. Battery level above minimum threshold
6. GPS fix adequate (outdoor) or MoCap tracking (indoor)
7. Geofence loaded and valid

### B.6.3 Takeoff Sequence (Pseudocode)

```
function takeoff(target_altitude_m):
    assert state == ARMED
    run_preflight_checks()

    state = TAKING_OFF
    request_takeoff_trajectory(target_altitude_m)
    command_offboard_mode()

    while altitude < target_altitude_m - tolerance:
        publish_feedback(current_altitude, progress_percent)
        if safety_alert or timeout:
            abort_and_land()
            return FAILURE

    state = HOVERING
    return SUCCESS
```

### B.6.4 Coordination with Other Managers

```
Startup ordering:
  1. hardware_abstraction  → connects to PX4
  2. estimation_manager    → loads estimator, waits for state
  3. control_manager       → loads controller
  4. trajectory_manager    → loads generators
  5. safety_monitor        → begins monitoring
  6. uav_manager           → runs self-check, publishes IDLE

During flight, uav_manager:
  - Monitors all manager /status topics
  - Responds to safety_monitor alerts
  - Handles action requests (takeoff, land, goto)
  - Publishes mode changes to all managers
  - Aggregates status for external consumers (GCS, TUI)
```

---

## B.7 Multi-Agent Architecture

### B.7.1 Current Status

| Component | Status |
|-----------|--------|
| Per-UAV namespace isolation | Implemented |
| DDS domain-based container isolation | Implemented |
| Zenoh bridge for inter-container topic sharing | Implemented |
| Multi-instance PX4 SITL launch | Implemented |
| GCS TUI observing multiple UAVs | Implemented |
| BVC collision avoidance | Planned |
| Consensus protocols | Planned |
| Decentralized coordination | Planned |

### B.7.2 Multi-Container Architecture (Implemented)

```
┌──────────────────────────────────────────────────────────┐
│  Container: sim                                          │
│  Shared Gazebo + N x PX4 SITL instances                 │
│  PX4 inst 0: domain_id=1, XRCE port 8888               │
│  PX4 inst 1: domain_id=2, XRCE port 8890               │
│  MicroXRCE Agent x2                                     │
└──────────┬──────────────────────────┬────────────────────┘
           │ DDS domain 1             │ DDS domain 2
┌──────────▼──────────┐    ┌──────────▼──────────┐
│  Container: uav1    │    │  Container: uav2    │
│  DOMAIN_ID=1        │    │  DOMAIN_ID=2        │
│  LOCALHOST_ONLY=1   │    │  LOCALHOST_ONLY=1   │
│  peregrine stack    │    │  peregrine stack    │
│  zenoh-bridge       │    │  zenoh-bridge       │
│  (exports topics)   │    │  (exports topics)   │
└──────────┬──────────┘    └──────────┬──────────┘
           │ Zenoh                    │ Zenoh
┌──────────▼──────────────────────────▼──────────┐
│  Container: gcs                                │
│  DOMAIN_ID=99                                  │
│  zenoh-bridge (imports all /uav* topics)       │
│  tui_status (observes fleet)                   │
└────────────────────────────────────────────────┘
```

**Key Design Decisions:**
- **DDS domain isolation**: Each UAV container has its own `ROS_DOMAIN_ID` — prevents accidental topic cross-talk
- **`ROS_LOCALHOST_ONLY=1`**: DDS stays within each container
- **Zenoh bridges**: Selectively export only status topics (8 per UAV) across containers
- **`network_mode: host`**: All containers share host network for Zenoh peer discovery
- **GCS domain 99**: Completely isolated, receives only bridged status — prevents accidental command injection

### B.7.3 Zenoh Bridge Configuration

Per-UAV bridge exports a controlled set of status topics:

```
Bridged topics (per UAV):
  - uav_state            (raw PX4 state)
  - estimated_state      (filtered state)
  - safety_status        (safety monitor output)
  - estimation_status    (estimator health)
  - control_status       (controller health)
  - trajectory_status    (trajectory progress)
  - status               (comprehensive UAV status)
  - gps_status           (GPS fix quality)
```

GCS bridge subscribes to `uav*/` prefix and republishes on its local DDS domain.

### B.7.4 Planned: Buffered Voronoi Cell Collision Avoidance

```
Concept:
  Each UAV computes its own Voronoi cell — the region closer to itself
  than to any neighbor. The cell is shrunk inward by a safety buffer.
  If each UAV stays in its buffered cell, minimum separation is guaranteed.

Algorithm (per UAV i):
  for each neighbor j:
      midpoint = (position_i + position_j) / 2
      normal = normalize(position_i - position_j)
      buffered_point = midpoint + normal * safety_buffer
      constraint: normal · x <= normal · buffered_point

  Publish constraints → trajectory_manager
  trajectory_manager projects setpoints onto feasible region
```

### B.7.5 Planned: Decentralized Extensions

Future work for multi-agent coordination:
- **Peer discovery** via DDS discovery or heartbeat protocol
- **Consensus protocols** (RAFT for leader election)
- **Distributed task allocation**
- **Partial connectivity** tolerance — subgroups operate independently
- **Reconvergence** when connectivity is restored

---

## B.8 Infrastructure and Deployment

### B.8.1 Repository Structure

```
peregrine/                          ← Git super-repo
├── src/
│   ├── peregrine_core/             ← Core flight stack (submodule)
│   │   ├── hardware_abstraction/
│   │   ├── frame_transforms/
│   │   ├── estimation_manager/
│   │   ├── control_manager/
│   │   ├── trajectory_manager/
│   │   ├── safety_monitor/
│   │   ├── uav_manager/
│   │   ├── peregrine_interfaces/
│   │   ├── peregrine_bringup/
│   │   ├── tui_status/
│   │   └── rviz_plugins/
│   ├── peregrine_app_nav/          ← App repos (submodules, future)
│   └── px4_msgs/                   ← PX4 message definitions (submodule)
├── docker/
│   ├── docker/                     ← Dockerfiles (sim, jetson, rpi5)
│   ├── compose/                    ← Compose stacks per target
│   ├── config/                     ← Entrypoints, Zenoh configs
│   └── Makefile                    ← Operational commands
└── docs/
```

### B.8.2 Docker Image Strategy

| Image | Platform | Features | Use Case |
|-------|----------|----------|----------|
| `ros2-px4-flight-simulation` | x86 + NVIDIA | CUDA, Gazebo, PX4 SITL, ROS2 | Development, simulation |
| `ros2-px4-flight-jetson` | arm64 | CUDA, TensorRT, ROS2 | Jetson Orin companion computer |
| `ros2-px4-flight-rpi5` | arm64 | Lightweight, headless, ROS2 | RPi5 companion computer |

All images:
- Mount full repo workspace into container (`/ros2_ws`)
- Run as non-root user mapped to host UID/GID
- Map `DRONE_ID` → `ROS_DOMAIN_ID`
- Optionally start MicroXRCE Agent

### B.8.3 Build and Run Commands

```bash
cd docker

# Simulation
make build-sim          # Build simulation image
make shell-sim          # Persistent dev shell
make dev                # Disposable shell (--rm)

# Hardware
make build-jetson       # Build Jetson image
DRONE_ID=1 make jetson  # Run on Jetson

make build-rpi5         # Build RPi5 image
DRONE_ID=1 make rpi5    # Run on RPi5

# Multi-UAV simulation
make multi-sitl         # 2-UAV sim with Zenoh bridging
```

### B.8.4 Hardware Deployment (Jetson)

```
systemd service for auto-start on boot:

[Service]
ExecStart=ros2 launch peregrine_bringup hardware.launch.py
Restart=on-failure

Performance optimization:
  sudo nvpmodel -m 0        # Max power mode
  sudo jetson_clocks        # Max clock speeds
  setcap cap_sys_nice+ep    # Real-time priority for control nodes
```

---

## B.9 Monitoring and Visualization

### B.9.1 TUI Status Display (ncurses)

Real-time terminal-based monitoring:

```
┌──────────────────────────────────────────────────────────────────────┐
│ PEREGRINE Flight Stack - UAV1 (Alpha)                     [HOVERING]│
├────────────────────────────────┬─────────────────────────────────────┤
│ STATE                          │ POSITION (ENU)                      │
│   Mode:     OFFBOARD           │   X:     12.45 m   (East)           │
│   Armed:    YES                │   Y:     -3.21 m   (North)          │
│   Uptime:   00:05:23           │   Z:     10.02 m   (Up)             │
├────────────────────────────────┼─────────────────────────────────────┤
│ BATTERY                        │ CONNECTIONS                         │
│   Voltage:  22.8 V             │   PX4:    ████████ OK               │
│   Percent:  78%  ████████░░    │   Est:    ████████ OK               │
├────────────────────────────────┼─────────────────────────────────────┤
│ MANAGERS                       │ SAFETY                              │
│   Estimator:   px4_passthrough │   Status:    NOMINAL                │
│   Controller:  se3_geometric   │   Geofence:  OK (42m to edge)      │
│   Trajectory:  waypoint_linear │   Battery:   OK                    │
├──────────────────────────────────────────────────────────────────────┤
│ ALERTS                                                               │
│   [INFO]  12:34:56  Takeoff complete, entering HOVERING              │
└──────────────────────────────────────────────────────────────────────┘
```

- Color-coded: green (nominal), yellow (warning), red (error/emergency)
- Multi-UAV compact view available for fleet monitoring
- Keyboard navigation for switching between UAVs

### B.9.2 RViz2 Flight Visualization

- 3D visualization of UAV position, orientation, trajectory
- Multi-UAV support with per-UAV markers
- Uses TF2 frame tree for positioning

---

# Part C: Code Overview

---

## C.1 Custom Interfaces (`peregrine_interfaces`)

### C.1.1 Design Principles

- **Single package** for all interface definitions — minimizes dependency coupling
- **No code/logic** in this package — only `.msg`, `.srv`, `.action` files
- **Semantic field names** with explicit units: `altitude_m`, `velocity_mps`, `angle_rad`
- **Frame explicitness**: all spatial messages include `frame_id`
- **All messages timestamped** via `std_msgs/Header`

### C.1.2 Core State Message

```
State.msg — Complete UAV state in ENU/FLU

  std_msgs/Header header
  string frame_id                        # "world" or "odom"

  # Position (ENU) [m]
  geometry_msgs/Point position           # x=East, y=North, z=Up

  # Velocity (ENU) [m/s]
  geometry_msgs/Vector3 velocity

  # Acceleration (ENU) [m/s^2]
  geometry_msgs/Vector3 acceleration

  # Orientation (ENU→FLU quaternion)
  geometry_msgs/Quaternion orientation

  # Angular velocity (FLU body frame) [rad/s]
  geometry_msgs/Vector3 angular_velocity

  # Euler angles (convenience, derived from quaternion)
  float64 roll_rad, pitch_rad, yaw_rad

  # Covariance (6x6 each, optional)
  float64[36] pose_covariance
  float64[36] twist_covariance

  # Metadata
  string estimator_name
  float64 confidence                     # [0.0 - 1.0]
```

### C.1.3 Control Output Message

```
ControlOutput.msg — Controller output to hardware_abstraction

  std_msgs/Header header
  string controller_name

  # Control mode (determines which fields are used)
  uint8 control_mode
    CONTROL_MODE_POSITION  = 0    # Position + yaw → PX4 position controller
    CONTROL_MODE_VELOCITY  = 1    # Velocity + yaw_rate → PX4 velocity controller
    CONTROL_MODE_ATTITUDE  = 2    # Quaternion + thrust → PX4 attitude controller
    CONTROL_MODE_BODY_RATE = 3    # Angular rates + thrust → PX4 rate controller

  # Fields populated based on control_mode:
  geometry_msgs/Point position
  float64 yaw_rad
  geometry_msgs/Vector3 velocity
  float64 yaw_rate_radps
  geometry_msgs/Quaternion orientation
  geometry_msgs/Vector3 body_rates
  float64 thrust_normalized              # [0.0 - 1.0]
```

### C.1.4 Trajectory Messages

```
TrajectorySetpoint.msg — Reference setpoint for controller

  std_msgs/Header header

  # Validity flags (which fields are populated)
  bool position_valid, velocity_valid, acceleration_valid
  bool jerk_valid, yaw_valid, yaw_rate_valid

  # Setpoint fields (ENU)
  geometry_msgs/Point position
  geometry_msgs/Vector3 velocity
  geometry_msgs/Vector3 acceleration
  geometry_msgs/Vector3 jerk
  float64 yaw_rad
  float64 yaw_rate_radps


Trajectory.msg — Complete trajectory

  string trajectory_id, generator_name

  uint8 trajectory_type
    TRAJECTORY_WAYPOINT    = 0
    TRAJECTORY_POLYNOMIAL  = 1
    TRAJECTORY_BSPLINE     = 2

  # Timing
  builtin_interfaces/Time start_time
  float64 total_duration_s

  # Waypoints (for WAYPOINT type)
  geometry_msgs/PoseStamped[] waypoints
  float64[] waypoint_times_s

  # Polynomial coefficients (for POLYNOMIAL type)
  float64[] poly_coeffs_x, poly_coeffs_y, poly_coeffs_z, poly_coeffs_yaw
  uint32 poly_order
```

### C.1.5 Safety Status Message

```
SafetyStatus.msg — Comprehensive safety state

  std_msgs/Header header

  uint8 safety_level
    LEVEL_NOMINAL   = 0
    LEVEL_WARNING   = 1
    LEVEL_CAUTION   = 2
    LEVEL_EMERGENCY = 3

  # Component health
  bool estimator_healthy, controller_healthy, trajectory_healthy
  bool px4_connected, gcs_connected

  # Geofence
  uint8 geofence_status (OK / WARNING / VIOLATION)
  float64 distance_to_geofence_m

  # Heartbeats
  uint8 heartbeats_alive, heartbeats_total
  string[] dead_heartbeats

  # Battery
  float64 battery_percent, battery_voltage_v
  uint8 battery_status (OK / WARNING / CRITICAL)

  # Active alerts
  string[] active_warnings, active_errors

  # Recommended action
  uint8 recommended_action (NONE / WARN / HOLD / RTH / LAND / EMERGENCY_STOP)
```

### C.1.6 Services and Actions

```
Services:
  SetController.srv      — Switch active controller plugin
  SetEstimator.srv       — Switch active estimator plugin
  SetTrajectoryGenerator — Switch active trajectory generator
  Arm.srv                — Arm/disarm UAV
  EmergencyStop.srv      — Emergency stop (requires confirm=true)

Actions:
  Takeoff.action         — Takeoff to altitude (feedback: current alt, progress)
  Land.action            — Land at position (feedback: altitude AGL, progress)
  SetTrajectory.action   — Execute trajectory (feedback: TrajectoryStatus)
  GoToPosition.action    — Go to position (feedback: distance remaining, ETA)
```

---

## C.2 Frame Transforms (`frame_transforms`)

### C.2.1 Purpose

Header-only library providing all coordinate frame conversions + a TF2 broadcaster node.

### C.2.2 Key Conversions (inline, zero-allocation)

```cpp
namespace peregrine::frame_transforms {

// Position: ENU ↔ NED
//   x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d& enu) {
    return {enu.y(), enu.x(), -enu.z()};
}

// Body frame: FLU ↔ FRD
//   x_frd = x_flu, y_frd = -y_flu, z_frd = -z_flu
inline Eigen::Vector3d flu_to_frd(const Eigen::Vector3d& flu) {
    return {flu.x(), -flu.y(), -flu.z()};
}

// Quaternion: ENU-FLU ↔ NED-FRD (simplified)
//   q_ned = (q.w, q.y, q.x, -q.z)
inline Eigen::Quaterniond quaternion_enu_to_ned_simple(
    const Eigen::Quaterniond& q_enu) {
    return {q_enu.w(), q_enu.y(), q_enu.x(), -q_enu.z()};
}

// Yaw: yaw_ned = π/2 - yaw_enu (then normalize to [-π, π])
inline double yaw_enu_to_ned(double yaw_enu);

// Covariance: C_ned = R * C_enu * R^T
//   where R = [0 1 0; 1 0 0; 0 0 -1]
inline Eigen::Matrix3d covariance_enu_to_ned(const Eigen::Matrix3d& cov_enu);
}
```

### C.2.3 Performance Considerations

- All conversion functions are `inline` — no function call overhead
- Eigen fixed-size types — no dynamic allocation
- Precomputed constants (`constexpr`) where possible
- Called at 250+ Hz in the control loop

### C.2.4 TF2 Broadcaster

```cpp
class UAVTFBroadcaster {
    // Broadcasts: world → odom → base_link → sensor_frames
    void broadcastState(const State& state);  // dynamic: pose updates
    void broadcastStaticSensorTransform(...); // static: camera, IMU offsets
};
```

---

## C.3 Hardware Abstraction (`hardware_abstraction`)

### C.3.1 Role

The sole interface between PEREGRINE and PX4. All other packages interact with PX4 only through this node.

### C.3.2 Bidirectional Translation

```
FROM PX4 (NED/FRD, px4_msgs) ──► hardware_abstraction ──► TO STACK (ENU/FLU, peregrine_interfaces)

  /fmu/out/vehicle_odometry    →  convert NED→ENU  →  /hardware_abstraction/px4_state
  /fmu/out/vehicle_status      →  parse arming/mode →  /hardware_abstraction/px4_status
  /fmu/out/battery_status      →  forward           →  /hardware_abstraction/battery

FROM STACK (ENU/FLU) ──► hardware_abstraction ──► TO PX4 (NED/FRD, px4_msgs)

  /control_manager/control_output  →  convert ENU→NED  →  /fmu/in/trajectory_setpoint
                                                        →  /fmu/in/vehicle_attitude_setpoint
                                                        →  /fmu/in/vehicle_rates_setpoint
                                   →  set flags         →  /fmu/in/offboard_control_mode
```

### C.3.3 PX4 QoS Requirements

```cpp
// PX4 requires specific QoS — wrong QoS = silent communication failure
auto px4_qos = rclcpp::QoS(5)
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
```

### C.3.4 Offboard Mode Management

```cpp
// Must publish OffboardControlMode at >= 2Hz or PX4 exits offboard
void publishOffboardControlMode() {
    OffboardControlMode msg;
    msg.timestamp = getTimestamp();

    // Set flags based on which control level is active
    switch (control_mode_) {
        case POSITION:  msg.position = true;  break;
        case VELOCITY:  msg.velocity = true;  break;
        case ATTITUDE:  msg.attitude = true;  break;
        case BODY_RATE: msg.body_rate = true; break;
    }
    offboard_pub_->publish(msg);
}
// Timer at 10Hz ensures this requirement is always met
```

### C.3.5 NaN Convention

```
PX4 uses NaN to indicate "field not used":
  - Position mode: velocity fields = NaN
  - Velocity mode: position fields = NaN
  - Zero is a VALID setpoint (hover at origin, zero velocity)
  - Setting unused fields to 0 instead of NaN causes bugs
```

### C.3.6 Time Synchronization

```
PX4 timestamps: microseconds since boot
ROS2 timestamps: nanoseconds since epoch

hardware_abstraction maintains a timestamp offset:
  offset = ros_now_us - px4_timestamp
  px4_timestamp = ros_now_us - offset

Updated from /fmu/out/timesync_status at 10Hz
```

---

## C.4 Estimation Manager (`estimation_manager`)

### C.4.1 Plugin Interface

```cpp
class EstimatorBase {
    virtual bool initialize(Node::SharedPtr node, string name) = 0;
    virtual bool activate() = 0;
    virtual bool deactivate() = 0;

    // Core processing
    virtual void processPX4State(const State& px4_state) = 0;
    virtual void injectExternalPose(const PoseStamped& pose) = 0;

    // Output
    virtual State getState() const = 0;

    // Health
    virtual bool isHealthy() const = 0;
    virtual double getConfidence() const = 0;  // [0.0 - 1.0]

    virtual void reset(const State* initial_state = nullptr) = 0;
};
```

### C.4.2 PX4 Passthrough Estimator (Default)

- Forwards PX4's EKF2 state estimate directly
- Optionally fuses external MoCap pose for position (configurable)
- External pose timeout: if MoCap data is stale (>50ms), falls back to PX4-only

```
Configuration:
  px4_passthrough:
    use_external_pose: false        # true for MoCap environments
    external_pose_timeout_s: 0.05   # 50ms timeout
```

### C.4.3 Manager Node

```
Subscriptions:
  /hardware_abstraction/px4_state    ← PX4 state at 250Hz
  /mocap/pose                        ← External MoCap (optional)

Publications:
  /estimation_manager/state          → Estimated state at 250Hz
  /estimation_manager/status         → Estimator health at 10Hz

Services:
  /estimation_manager/set_estimator  → Switch active estimator plugin
```

### C.4.4 Adding a Custom Estimator

```cpp
// 1. Implement EstimatorBase
class MyCustomEKF : public EstimatorBase {
    void processPX4State(const State& state) override {
        // Your EKF prediction + update logic
    }
    State getState() const override { return estimated_state_; }
    // ... other methods
};

// 2. Register plugin
PLUGINLIB_EXPORT_CLASS(MyCustomEKF, EstimatorBase)

// 3. Add to config YAML
estimator_manager:
  plugins: ["px4_passthrough", "my_custom_ekf"]
```

---

## C.5 Control Manager (`control_manager`)

### C.5.1 Controller Plugin Interface

```cpp
class ControllerBase {
    virtual bool initialize(Node::SharedPtr node, string name) = 0;
    virtual bool activate() = 0;
    virtual bool deactivate() = 0;

    // Core: compute control output from state + reference
    virtual ControlOutput compute(
        const State& current_state,
        const TrajectorySetpoint& reference,
        double dt) = 0;

    // Introspection
    virtual ControlMode getControlMode() const = 0;  // POSITION/VELOCITY/ATTITUDE/BODY_RATE
    virtual bool isHealthy() const = 0;
    virtual map<string, double> getParameters() const = 0;
    virtual bool setParameters(const map<string, double>& params) = 0;
};
```

### C.5.2 Control Hierarchy — Where PEREGRINE Meets PX4

```
                    PEREGRINE computes        PX4 handles the rest
                    ─────────────────         ───────────────────
POSITION mode:      [position setpoint] ──►  PX4 Pos → Vel → Att → Rate → Motors
VELOCITY mode:      [velocity setpoint] ──►  PX4 Vel → Att → Rate → Motors
ATTITUDE mode:      [quaternion+thrust] ──►  PX4 Att → Rate → Motors
BODY_RATE mode:     [rates + thrust]    ──►  PX4 Rate → Motors

Lower modes = more control authority for PEREGRINE
Higher modes = more PX4 does, simpler companion code
```

### C.5.3 PX4 Passthrough Controller

Simplest controller — passes trajectory setpoints directly to PX4:

```cpp
ControlOutput compute(const State& state, const TrajectorySetpoint& ref, double dt) {
    ControlOutput output;
    output.control_mode = CONTROL_MODE_POSITION;  // or VELOCITY
    output.position = ref.position;
    output.yaw_rad = ref.yaw_rad;
    return output;
}
```

### C.5.4 SE(3) Geometric Controller

Full geometric tracking controller on SE(3):

```
Reference: Lee, Leok, McClamroch — "Geometric Tracking Control of a Quadrotor UAV on SE(3)"

Algorithm:
  1. Compute position error:   e_pos = position - position_desired
  2. Compute velocity error:   e_vel = velocity - velocity_desired
  3. Desired acceleration:     a_des = -Kp * e_pos - Kd * e_vel + a_ff - g
  4. Desired thrust direction: z_des = normalize(a_des)
  5. Desired thrust magnitude: F = mass * dot(a_des, body_z_axis)
  6. Construct desired rotation matrix from z_des + desired yaw
  7. Output: desired quaternion + normalized thrust

Output mode: ATTITUDE (quaternion + thrust → PX4 attitude controller)

Configurable gains:
  kp_pos: [6.0, 6.0, 8.0]      # Position proportional gains (x, y, z)
  kd_pos: [4.5, 4.5, 5.0]      # Position derivative gains
  mass: 1.5                      # Vehicle mass [kg]
  max_thrust: 30.0               # Maximum thrust [N]
```

### C.5.5 Control Loop

```cpp
// Runs at 250Hz via timer
void controlLoop() {
    if (!active_controller_ || !state_received_) return;

    double dt = compute_dt();  // with sanity check

    // Default to holding current position if no trajectory reference
    TrajectorySetpoint reference = reference_received_
        ? current_reference_
        : hold_current_position(current_state_);

    auto output = active_controller_->compute(current_state_, reference, dt);
    control_output_pub_->publish(output);
}
```

---

## C.6 Trajectory Manager (`trajectory_manager`)

### C.6.1 Trajectory Generator Plugin Interface

```cpp
class TrajectoryGeneratorBase {
    virtual bool initialize(Node::SharedPtr node, string name) = 0;

    // Generate trajectory from request
    virtual Trajectory generate(const TrajectoryRequest& request) = 0;

    // Evaluate trajectory at time t → setpoint
    virtual TrajectorySetpoint evaluate(const Trajectory& traj, double t) = 0;

    // Output capability
    virtual OutputType getOutputType() const = 0;
    // POSITION_ONLY: x, y, z
    // POSITION_YAW:  x, y, z, yaw
    // FULL_STATE:    pos, vel, acc, jerk, yaw, yaw_rate
};
```

### C.6.2 Setpoint Publishing Loop

```
On trajectory start:
  start_time = now()

Every tick (50Hz):
  elapsed = now() - start_time
  setpoint = generator.evaluate(trajectory, elapsed)

  // Check collision constraints (from multi_agent_coordinator)
  for each constraint:
      if violates(setpoint, constraint):
          setpoint = project_onto_boundary(setpoint, constraint)

  publish(setpoint)

  if elapsed >= trajectory.total_duration:
      mark_completed()
```

### C.6.3 Available Generators

| Generator | Output | Use Case |
|-----------|--------|----------|
| `WaypointLinearGenerator` | POSITION_YAW | Linear interpolation between waypoints |
| `HoldPositionGenerator` | POSITION_YAW | Hovering, emergency hold |
| `TakeoffLandGenerator` | POSITION_YAW | Vertical takeoff/landing with S-curve |
| `PolynomialGenerator` (planned) | FULL_STATE | Minimum-snap trajectories for aggressive flight |

### C.6.4 Action Interface

```
Action: SetTrajectory
  Goal:     Trajectory to execute, replace_current flag
  Feedback: TrajectoryStatus (progress %, position error, elapsed time)
  Result:   success/fail, completion_status

Action: GoToPosition
  Goal:     target position (ENU), yaw, velocity, acceptance radius
  Feedback: distance_remaining_m, eta_s
  Result:   success/fail, final_position
```

---

## C.7 Safety Monitor (`safety_monitor`)

### C.7.1 Architecture

```
safety_monitor_node
    │
    ├── SafetyCheckerBase (interface)
    │       │
    │       ├── GeofenceChecker     ← cylindrical/polygonal geofence
    │       ├── BatteryChecker      ← 3-threshold battery monitoring
    │       ├── EnvelopeChecker     ← velocity, tilt, altitude limits
    │       ├── GPSChecker          ← fix quality, HDOP
    │       └── (extensible)
    │
    ├── RuleEngine                  ← evaluates checker outputs, debouncing
    │
    └── SafetyActionExecutor        ← maps alerts to recommended actions
```

### C.7.2 Checker Interface

```cpp
class SafetyCheckerBase {
    virtual string getName() const = 0;
    virtual SafetyLevel check(const State& state, ...) = 0;
    // Returns: NOMINAL, WARNING, CAUTION, or EMERGENCY
};
```

### C.7.3 Geofence Implementation

```
Cylindrical geofence:
  distance_horizontal = sqrt(x^2 + y^2)  // from home
  distance_vertical = z

  if distance_horizontal > max_radius - critical_buffer:
      return EMERGENCY
  elif distance_horizontal > max_radius - warning_buffer:
      return WARNING
  else:
      return NOMINAL

  // Same logic for altitude limits

Parameters:
  max_radius_m: 100.0
  max_altitude_m: 50.0
  min_altitude_m: 0.5
  warning_buffer_m: 10.0
  critical_buffer_m: 5.0
```

### C.7.4 Rule Engine

- Evaluates all checkers at 100 Hz
- Implements debouncing: single bad reading doesn't trigger emergency
- Aggregates checker outputs into overall safety level
- Rate-limits repeated alerts to prevent alert storms

### C.7.5 Configuration

```yaml
safety_monitor:
  ros__parameters:
    check_rate_hz: 100.0
    geofence:
      type: "cylindrical"
      max_radius_m: 100.0
      max_altitude_m: 50.0
      violation_action: "land"
    heartbeats:
      hardware_abstraction: { timeout_s: 1.0, priority: "critical" }
      estimator_manager:    { timeout_s: 0.5, priority: "critical" }
      controller_manager:   { timeout_s: 0.5, priority: "high" }
    envelope:
      max_velocity_mps: 10.0
      max_tilt_rad: 0.7
    battery:
      warning_percent: 30.0
      critical_percent: 20.0
      emergency_percent: 10.0
```

---

## C.8 TUI Status Display (`tui_status`)

### C.8.1 Implementation

- ncurses-based terminal UI
- Subscribes to status topics from all managers
- 10 Hz refresh rate with non-blocking keyboard input
- Color-coded status (green/yellow/red)
- Alert buffer (ring buffer, 100 entries)

### C.8.2 Display Sections

| Section | Data Source | Content |
|---------|-----------|---------|
| State | `uav_manager/status` | Mode, armed, uptime |
| Position | `estimation_manager/state` | x, y, z (ENU), yaw |
| Velocity | `estimation_manager/state` | vx, vy, vz, speed |
| Attitude | `estimation_manager/state` | roll, pitch, yaw, tilt |
| Battery | `hardware_abstraction/battery` | voltage, percent, bar graph |
| Connections | heartbeat status | PX4, GCS, estimator, controller |
| Managers | various /status topics | active plugin names, trajectory progress |
| Safety | `safety_monitor/safety_status` | level, geofence distance, battery |
| Alerts | `safety_monitor/alerts` | timestamped alert log |

### C.8.3 Multi-UAV View

Compact table format showing all UAVs:

```
│  ID    │  State  │ Position │ Battery │ Safety  │ Status      │
│  UAV1  │ HOVER   │ 12,-3,10 │   78%   │ NOMINAL │ Traj: 45%   │
│  UAV2  │ FLYING  │  8, 5,12 │   82%   │ NOMINAL │ Traj: 67%   │
```

---

## C.9 Launch and Configuration (`peregrine_bringup`)

### C.9.1 Launch File Hierarchy

```
simulation.launch.py         ← Gazebo + PX4 SITL + stack
  └── single_uav_sitl.launch.py  ← PX4 SITL + stack for one UAV
      └── single_uav.launch.py   ← Stack only (no SITL)

multi_uav_sitl.launch.py     ← N x PX4 SITL instances in shared Gazebo
  (UAV containers launch single_uav.launch.py independently)
```

### C.9.2 Single UAV Launch

Launches all manager nodes in order with shared config:

```python
# Startup order matters:
1. hardware_abstraction    # Connects to PX4 first
2. frame_transforms        # TF broadcasting
3. estimation_manager      # Needs PX4 state
4. control_manager         # Needs estimated state
5. trajectory_manager      # Needs controller ready
6. safety_monitor          # Monitors everything
7. uav_manager             # Coordinates all (last)
```

### C.9.3 Configuration Structure

```
peregrine_bringup/config/
├── default.yaml                  # Base configuration (all nodes)
├── environments/
│   ├── gps.yaml                  # GPS/outdoor overrides
│   ├── mocap.yaml                # MoCap/indoor overrides
│   └── simulation.yaml           # Simulation-specific
├── uav_types/
│   ├── quadrotor_x.yaml          # X-frame quadrotor params
│   └── hexarotor.yaml            # Hexarotor params
└── fleet/
    └── 4_uav_diamond.yaml        # Fleet formation params
```

### C.9.4 Key Parameters (default.yaml excerpt)

```yaml
hardware_abstraction:
  ros__parameters:
    offboard_rate_hz: 10.0           # Offboard heartbeat rate

estimator_manager:
  ros__parameters:
    publish_rate_hz: 250.0
    default_estimator: "px4_passthrough"

controller_manager:
  ros__parameters:
    control_rate_hz: 250.0
    default_controller: "px4_passthrough"

trajectory_manager:
  ros__parameters:
    publish_rate_hz: 50.0
    default_generator: "waypoint_linear"

uav_manager:
  ros__parameters:
    default_takeoff_altitude_m: 2.0

safety_monitor:
  ros__parameters:
    check_rate_hz: 100.0
    geofence:
      max_radius_m: 100.0
      max_altitude_m: 50.0
```

---

## C.10 Implementation Status Summary

| Package | Status | Notes |
|---------|--------|-------|
| `peregrine_interfaces` | Implemented | All messages, services, actions defined |
| `frame_transforms` | Implemented | Conversions + TF broadcaster, unit tested |
| `hardware_abstraction` | Implemented | PX4 bridge, all control modes, time sync |
| `estimation_manager` | Implemented | Manager + PX4 passthrough plugin |
| `control_manager` | Implemented | Manager + PX4 passthrough + SE(3) geometric controller |
| `trajectory_manager` | Implemented | Manager + waypoint linear + hold + takeoff/land |
| `safety_monitor` | Implemented | Geofence, battery, envelope, GPS, rule engine |
| `uav_manager` | Implemented | State machine, preflight, takeoff/land sequences |
| `tui_status` | Implemented | Single + multi-UAV views, ncurses |
| `rviz_plugins` | Implemented | Flight visualization node |
| `peregrine_bringup` | Implemented | Single UAV, multi-UAV SITL, configs |
| Docker stack | Implemented | Sim, Jetson, RPi5 images + compose |
| Zenoh bridging | Implemented | Inter-container topic sharing |
| `multi_agent_coordinator` | Planned | BVC, consensus, decentralized coordination |
| Polynomial trajectories | Planned | Minimum-snap trajectory generation |
| Custom EKF estimator | Planned | Multi-sensor fusion |
