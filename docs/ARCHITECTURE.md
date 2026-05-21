# Peregrine Architecture: Separation of Concerns

This document describes the layered architecture of the Peregrine flight stack, the temporal hierarchy, and the design decisions behind process boundaries.

---

## Temporal Hierarchy

The stack operates at four distinct frequency tiers. Each layer is independent — a failure or stall at a slower layer does not affect faster layers.

| Layer | Rate | Owner | Responsibility |
|-------|------|-------|----------------|
| Hard RT | 250 Hz | `hardware_abstraction` | PX4 ↔ peregrine message translation, actuator output, sensor ingestion |
| Soft RT | 50 Hz | `control_manager`, `trajectory_manager`, `estimation_manager` | Control allocation, trajectory sampling, state estimation |
| Reflexes | 2–10 Hz | `safety_monitor`, `uav_manager` FSM | Rule engine evaluation, safety level computation, flight state transitions |
| Application | 1–10 Hz | BT executor (`peregrine_bt`) | Mission sequencing, recovery logic, operator intent |

Key invariant: **slower layers never bypass faster ones.** The BT does not send raw PX4 commands — it calls `uav_manager` actions and `trajectory_manager` goals. The FSM does not write actuator outputs — it sets modes and gates transitions. Each layer trusts the layer below it to handle timing-critical concerns.

---

## FSM vs BT: Who Owns What

### The FSM (`uav_manager`) owns:

- **Flight state machine** — `LANDED_DISARMED → LANDED_ARMED → TAKING_OFF → FLYING → LANDING → LANDED_ARMED → LANDED_DISARMED`
- **Safety gates** — refuses Takeoff if dependencies aren't ready, refuses actions during emergency
- **State transitions** — Arm, Takeoff, Land are FSM events that must go through `uav_manager`
- **Emergency latching** — when `safety_monitor` escalates to EMERGENCY, the FSM latches it and requires explicit `ClearEmergency` to recover
- **Recovery actions** — if takeoff fails with vehicle armed, FSM initiates recovery land

The FSM is **reactive and stateful**. It doesn't know about missions, trajectories, or waypoints. It knows the vehicle is armed, flying, or landed, and it enforces invariants.

### The BT (`peregrine_bt`) owns:

- **Mission sequencing** — what to do and in what order (arm → takeoff → circle → land)
- **Recovery logic** — what to do when something fails (retry trajectory, abort mission, emergency land)
- **Conditional behavior** — if battery < 20%, skip remaining waypoints and land
- **Operator intent** — the tree XML is the mission definition, authored by the operator

The BT is **deliberative and stateless** (between missions). It reads world state from topics, makes decisions, and issues commands via actions/services. It does not hold flight state — that's the FSM's job.

### The boundary:

```
BT: "I want to take off to 5m"
  → calls uav_manager/takeoff action
    → FSM validates: dependencies ready? not in emergency? not already flying?
      → FSM executes: arm → set offboard → send trajectory goal
        → trajectory_manager generates climb trajectory
          → control_manager tracks it at 50Hz
            → hardware_abstraction sends actuator commands at 250Hz
```

The BT expresses intent. The FSM validates and executes. The pipeline delivers. Each layer can reject — the BT must handle rejection gracefully (that's what reactive fallbacks and retry nodes are for).

---

## Process Boundaries

### Core flight stack: single process

The 7 core nodes run as composable nodes in one `component_container_mt` process:

- `hardware_abstraction`
- `frame_transforms`
- `estimation_manager`
- `control_manager`
- `trajectory_manager`
- `safety_monitor`
- `uav_manager`

**Why single process:** These nodes form a tightly-coupled real-time data pipeline. Intra-process communication gives zero-copy message passing on the 50–250 Hz data paths. On resource-constrained hardware (Jetson Orin Nano, RPi5), minimizing process count reduces memory overhead and context-switch latency.

**Why these 7 and no more:** Every node in this container is part of the flight-critical path. A crash in any of them is already a flight-critical event. Adding non-flight-critical code to this container increases the crash surface without benefit.

### BT executor: separate process

The BT executor runs as its own `Node` (not `ComposableNode`), launched from `bt_mission.launch.py` which includes `core_stack.launch.py`.

**Why separate process:**

1. **Fault isolation.** The BT is the most user-facing component — operators author tree XML, swap missions, and may load custom BT plugins. A segfault in a bad plugin or a logic bug in a tree would, if composable in the core container, take down the entire flight stack — including `safety_monitor` and the FSM. With process isolation, a BT crash leaves the flight stack running. The FSM holds position (or lands, depending on safety level) autonomously.

2. **Independent lifecycle.** Loading a new mission tree should not require restarting the flight stack. As a separate process, the BT node can be deactivated, reconfigured with a new tree file, and reactivated while the core stack continues operating.

3. **No performance cost.** The BT communicates with the stack via:
   - Topic subscriptions at ~10–50 Hz (state monitoring) — inter-process serialization overhead is microseconds, irrelevant at these rates
   - Action goals (takeoff, land, execute_trajectory) — multi-second operations where serialization is invisible
   - Services (arm, clear_emergency) — same, negligible overhead
   
   There is no high-frequency data path between the BT and the core stack that would benefit from intra-process zero-copy.

4. **Thread isolation.** The BT tick callback (1–10 Hz) does not compete with the 50–250 Hz manager callbacks for executor thread pool time.

5. **Industry precedent.** Nav2 runs `bt_navigator` as its own node, separate from controller/planner servers. MoveIt2's task constructor does the same. Separate-process BT executor is the established ROS 2 pattern.

### Deployment topology

**Hardware (Jetson / RPi5):**
```
Docker container "aircraft"
├── Process 1: component_container_mt (core stack — 7 nodes)
├── Process 2: bt_executor (if mission is loaded)
└── Process 3: MicroXRCEAgent (PX4 bridge)
```

**SITL (single UAV):**
```
Docker container "sim"
├── PX4 SITL instance
└── Gazebo

Docker container "uav1"
├── Process 1: component_container_mt (core stack)
├── Process 2: bt_executor (optional)
└── Process 3: MicroXRCEAgent
```

**SITL (multi UAV):**
Each UAV gets its own container with independent core stack and optional BT executor, isolated by `ROS_DOMAIN_ID`.

---

## Design Rules for New Components

1. **If it's flight-critical and runs at ≥10 Hz** → composable node in the core stack container.
2. **If it's application-layer or operator-facing** → separate process.
3. **If it processes high-frequency data from the core stack** → composable node (benefits from zero-copy).
4. **If it only reads state topics and issues action/service calls** → separate process (no zero-copy benefit, gains fault isolation).
5. **If a crash in this component should not affect flight safety** → separate process (by definition).

The BT executor satisfies rules 2, 4, and 5. It belongs outside the core container.
