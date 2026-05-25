# BT & Mission Dispatch TODO

Status: Phase 1 complete, Phase 2 next
Last updated: 2026-05-25

## Architecture summary

Each UAV runs a `TreeExecutionServer` (standard `behaviortree_ros2` pattern)
that exposes an `ExecuteTree` action. The GCS sends a goal with a tree name
and a JSON payload for initial parameters. Feedback streams back during
execution. Cancellation halts the tree.

A lightweight GCS-side fleet bridge translates operator commands into
`ExecuteTree` goals and relays data. The complexity lives on the GCS, not
the UAV. New mission types = new tree XML, no UAV code changes.

**Safety architecture:** `uav_manager` independently enforces safety via
its state machine (FailsafeDetected → Emergency), goal rejection, and
mid-flight preemption. BT trees do NOT need reactive safety monitoring —
if something goes wrong, `uav_manager` overrides regardless of what the
tree is doing. Safety condition nodes (`IsSafetyNominal`, `IsSafetyAtLeast`)
are kept as optional preflight gates but should not be used in reactive
wrappers during flight.

**Dynamic mission data** (waypoints, targets, search areas changing mid-
flight) flows via ROS topics into the tree using `RosTopicSubNode` — the
same pattern used by all condition nodes. Thread safety is guaranteed
because `RosTopicSubNode` spins its own callback group per tick.

## Design principles

- **ROS-native data injection**: use `RosTopicSubNode` for getting data
  into BT trees, not `onLoopAfterTick()` or tick-loop callbacks.
- **TUI direct commands bypass BT**: arm, land, takeoff call flight stack
  services/actions directly. BT is for missions, not basic operations.
- **Application-independent core**: the executor, TUI commands, and fleet
  bridge are generic. Application-specific logic lives only in tree XMLs
  and optional new `RosTopicSubNode` subclasses.
- **Tree XMLs live on the UAV**: the GCS knows tree names, not tree files.

---

## Phase 1: TreeExecutionServer ✓ COMPLETE

Replaced `BTExecutorNode` (lifecycle node + 2Hz tick timer + dual-node
pattern) with `PeregrineTreeServer` (subclass of `BT::TreeExecutionServer`).

### What was done

- **`peregrine_tree_server.cpp`** replaces `bt_executor_node.cpp`.
  Subclasses `TreeExecutionServer`, overrides `registerNodesIntoFactory()`
  to call `registerAllNodes()`. Uses `MultiThreadedExecutor` with 250ms
  timeout (deadlock workaround from behaviortree_ros2 sample).

- **`tree_server.yaml`** configures the action server: 10Hz tick,
  tree directories `peregrine_bt/trees` and `peregrine_bt/trees/subtrees`.
  `plugins` key intentionally omitted (empty array causes
  `generate_parameter_library` crash).

- **`bt_mission.launch.py`** updated: regular `Node` instead of
  `LifecycleNode`, loads `tree_server.yaml`, node name `bt_action_server`.

- **`WaitUntilReady`** (StatefulActionNode): subscribes to `uav_state`,
  returns RUNNING until `dependencies_ready` + configurable settle time.
  Replaces the broken `RetryUntilSuccessful` + condition pattern (BT.CPP
  v4's `RetryUntilSuccessful` retries synchronous children within a single
  tick, burning through all attempts instantly).

- **SITL regression harness** updated to send `ExecuteTree` action goals
  instead of lifecycle transitions.

### What was removed

- `bt_executor_node.cpp` — custom lifecycle executor
- `circle_with_safety.xml` — reactive safety monitoring is redundant
  (uav_manager handles it independently)
- `controller_switch_demo.xml` — controller switching requires lifecycle
  transitions on `control_manager`, not a simple service call
- `SetModeService` BT node — only consumer was the deleted controller
  switch tree

### Current BT node inventory

**Condition nodes** (RosTopicSubNode — return SUCCESS/FAILURE, never RUNNING):
- UAV state (`uav_state`): `IsArmed`, `IsFlying`, `IsLanded`,
  `IsDependenciesReady`, `IsEmergency`, `IsConnected`, `IsOffboard`
- Safety (`safety_status`): `IsSafetyNominal`, `IsSafetyAtLeast`
- Sensors: `IsBatteryAbove` (`status`), `IsGpsHealthy` (`gps_status`)
- Position (`state`): `HasValidState`, `IsAboveAltitude`, `IsBelowAltitude`,
  `IsAtPosition`

**Action nodes** (RosActionNode — return RUNNING while waiting):
- `TakeoffAction` → `uav_manager/takeoff`
- `LandAction` → `uav_manager/land`
- `ExecuteTrajectoryAction` → `trajectory_manager/execute_trajectory`
- `GoToAction` → `trajectory_manager/go_to`

**Service nodes** (RosServiceNode):
- `ArmService` → `arm`
- `ClearEmergencyService` → `uav_manager/clear_emergency`

**Utility nodes** (StatefulActionNode — return RUNNING):
- `WaitAction` — waits for a duration
- `WaitUntilReady` — waits for `dependencies_ready` on `uav_state`

### Current trees

- `takeoff_hover_land.xml` — basic test: preflight → takeoff → hover → land
- `multi_trajectory.xml` — preflight → takeoff → circle → figure8 → land
- `subtrees/preflight_and_takeoff.xml` — shared subtree: WaitUntilReady → TakeoffAction
- `subtrees/preflight_checks.xml` — standalone WaitUntilReady

### SITL test results (last run)

- `bt/takeoff_hover_land`: **PASS**
- `bt/multi_trajectory`: **PASS**

---

## Phase 2: TUI direct commands

The TUI (`tui_status`) is currently read-only. Add keybinding-driven
commands for basic UAV operations. These call the flight stack directly —
they do NOT go through BT.

### Existing services/actions available

| Name                              | Type    | Interface                            |
|-----------------------------------|---------|--------------------------------------|
| `arm`                             | service | `peregrine_interfaces/srv/Arm`       |
| `uav_manager/clear_emergency`     | service | `peregrine_interfaces/srv/ClearEmergency` |
| `uav_manager/takeoff`             | action  | `peregrine_interfaces/action/Takeoff`|
| `uav_manager/land`                | action  | `peregrine_interfaces/action/Land`   |

### 2.1 Add service/action clients to TuiNode

- Create clients for the services and actions listed above.
- Use the same `topicName()` namespacing helper so it works in multi-UAV.

### 2.2 Add keybindings

| Key | Action                  | Call                          |
|-----|-------------------------|-------------------------------|
| `a` | Arm                     | `srv::Arm{arm: true}`         |
| `d` | Disarm                  | `srv::Arm{arm: false}`        |
| `t` | Takeoff (default 5 m)   | `action::Takeoff{alt: 5.0}`   |
| `l` | Land                    | `action::Land{vel: 0.8}`      |
| `e` | Clear emergency         | `srv::ClearEmergency{}`       |

- Show a confirmation prompt for arm/disarm (safety-critical).
- Disable arm/takeoff keys when already armed/flying.

### 2.3 Display command state

- Add a status line showing last command sent and result.
- Action feedback shown in alert buffer.

---

## Phase 3: TUI mission dispatch

Add the ability to send `ExecuteTree` action goals from the TUI.

### 3.1 Add ExecuteTree action client to TuiNode

- Single `rclcpp_action::Client<btcpp_ros2_interfaces::action::ExecuteTree>`.
- Namespaced via `topicName("execute_tree")`.

### 3.2 Mission selection UI

- Keybinding (`m`) opens mission picker overlay.
- Lists available trees (hardcoded initially, later queryable).
- User selects tree, optionally enters parameters.
- TUI sends `ExecuteTree` goal.

### 3.3 Mission monitoring

- Show current mission name and status in status display.
- Action feedback shown in alert buffer.
- Keybinding (`x`) cancels active mission.

---

## Phase 4: GCS fleet bridge (multi-UAV)

A Python ROS node for fleet-level mission management. Only needed when
operating multiple UAVs from a single operator station.

### 4.1 Fleet discovery and state tracking

- Discover UAVs by scanning for `/<ns>/execute_tree` action servers.
- Subscribe to each UAV's `uav_state` for fleet overview.
- Maintain per-UAV state: idle / mission-running / emergency / offline.

### 4.2 Mission dispatch

- Expose a service for the TUI to call:
  `AssignMission(uav_namespace, tree_name, payload_json) → success, message`
- Tracks active missions, relays feedback.

### 4.3 Mission data publishing

- For dynamic mid-flight data (waypoints, targets, search areas),
  fleet bridge publishes to per-UAV topics.
- BT tree reads via `RosTopicSubNode` nodes.

### 4.4 Multi-UAV TUI

- Fleet overview (multiple UAV panels or summary view).
- UAV selection (switch which UAV keybindings control).
- Fleet-wide commands (land all, abort all).

---

## Phase 5: Mission-specific trees and nodes

Application-specific work. Adding a new mission type means:
1. A new tree XML in `peregrine_bt/trees/`.
2. (Optional) new `RosTopicSubNode` subclass for new data types.
3. (Optional) new message type if no existing one fits.

No changes to the core executor, TUI, or fleet bridge.

---

## Dependencies

- `behaviortree_ros2` submodule provides `TreeExecutionServer` and
  `btcpp_ros2_interfaces/action/ExecuteTree`.
- Phases are sequential: 1 ✓ → 2 → 3 → 4. Phase 5 can happen in
  parallel with 3 or 4.
