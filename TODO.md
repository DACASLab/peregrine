# Peregrine Core: Refactoring & Improvement TODO

This document is the single source of truth for all planned refactoring work on the `peregrine_core` stack, organized by priority and phase. It combines the original architecture refactoring guide with findings from a comprehensive file-by-file code audit.

---

## Phase 0: Safety-Critical Fixes

These are correctness and safety bugs that should be fixed before any new feature work.

### 0.1 ~~Fix trajectory completion detection for curved generators~~
- **Status:** Superseded — completion detection was removed from generators entirely. Generators are now pure reference sources that output `(setpoint, progress)`. The trajectory_manager action server uses `progress >= 1.0` to succeed goals. No position-based completion exists anywhere in this layer; the orchestrator (BT or demo script) owns that decision if needed.
- [x] Removed `completed` and `distanceRemaining` from `TrajectorySample`
- [x] Removed `currentState` parameter from `sample()` — generators no longer inspect vehicle state
- [x] All generators report progress [0,1] based on planned time/motion only
- [x] trajectory_manager action server succeeds goals on `progress >= 1.0`

### 0.2 ~~Add safety_level freshness check to emergency auto-clear~~
- **Status:** Resolved by Phase 1.3 — emergency auto-clear was removed entirely. Recovery now requires explicit `ClearEmergency` service call.
- [x] Resolved (auto-clear removed in Phase 1.3)

### 0.3 Fix mutex scope in trajectory publish loop
- **Package:** `trajectory_manager` — `trajectory_manager_node.cpp`, `publishTrajectorySetpoint()`
- **Problem:** `dataMutex_` is held for 78+ lines covering trajectory sampling, setpoint construction, feedback publishing, AND goal resolution (`succeed()`/`abort()` calls). At 50Hz, subscriber callbacks can be blocked for the entire cycle.
- **Fix:** Split into three phases:
  1. `sampleActiveTrajectory()` — hold lock, copy state + sample result
  2. `emitSetpointAndFeedback()` — no lock, publish setpoint and feedback
  3. `resolveGoalCompletion()` — no lock, call succeed/cancel on goal handles
- [x] Refactor `publishTrajectorySetpoint()` into the three phases above
- [x] Verify no data races after refactor (goal handle access, generator swap)

### 0.4 Fix race condition in trajectory goal acceptance
- **Package:** `trajectory_manager` — `trajectory_manager_node.cpp`
- **Problem:** Generator is created in `onExecuteGoal()` for validation, then created again in `onExecuteAccepted()` with a different `this->now()` and potentially different `latestState_`. Vehicle state can change between the two calls.
- **Fix:** Cache the validated generator or the validated state snapshot during goal evaluation, reuse in `onExecuteAccepted()`.
- [x] Store validated generator in a pending slot during `onExecuteGoal()`
- [x] Consume it in `onExecuteAccepted()` instead of recreating

---

## Phase 1: uav_manager — Strip Down to State Gateway

Prepare `uav_manager` for a Behavior Tree application layer by removing orchestration logic and making it a pure state machine + safety gateway.

### 1.1 Remove GoTo and ExecuteTrajectory forwarding
- **Problem:** `uav_manager` proxies `GoTo` and `ExecuteTrajectory` actions by forwarding them to `trajectory_manager`. The BT will call `trajectory_manager` directly.
- **Keep:** `Takeoff`, `Land`, `Arm` (these are state transitions the FSM must own).
- [x] Delete `forwardGoTo()` (~87 LOC)
- [x] Delete `forwardExecuteTrajectory()` — renamed to `forwardTakeoffTrajectory()` (internal use only for takeoff)
- [x] Remove the GoTo and ExecuteTrajectory action servers from `uav_manager`
- [x] Remove the corresponding action client members (kept trajectoryExecuteClient_ for internal takeoff)
- [x] Audit GoTo.action — still used by trajectory_manager (serves GoTo directly) and safety_regression_demo. Removed `acceptance_radius_m` field; feedback changed from `distance_remaining_m` to `progress`.

### 1.1b Cleanup: delete dead ActionOrchestrator, add recovery land
- [x] Delete `action_orchestrator.cpp` and `action_orchestrator.hpp` — dead code after forwarding removal
- [x] Rename types to `step_result.hpp` (`StepCode` enum + `StepResult` struct — still used by `callArmService`/`callSetModeService`)
- [x] Remove `action_orchestrator.cpp` from `CMakeLists.txt`
- [x] Add `recoveryLand()` to `uav_manager` — sends PX4 land mode when takeoff fails with vehicle armed
- [x] Fix takeoff `failGoal` to call `recoveryLand` instead of leaving quad armed and airborne
- [x] Fix land auto-disarm timeout: treat as landed (not as failure → hovering) since PX4 land mode is already active

### 1.2 Remove `ensureArmableMode()`
- **Problem:** Forces PX4 into `POSCTL` when nav state would prevent arming. This hides bad state from the caller.
- **Fix:** If PX4 rejects arm, `uav_manager` must fail the action. Let the BT handle recovery.
- [x] Delete `ensureArmableMode()` and its call in the takeoff sequence
- [x] Update `Takeoff` action to return a clear error when PX4 rejects arm due to nav state
- [ ] Document expected pre-arm states in the action definition or README

### 1.3 Remove emergency auto-clear timer
- **Problem:** 5-second automatic failsafe clearance. Emergencies are latched faults — automatic clearance is dangerous.
- **Fix:** Expose a `ClearEmergency` ROS service. Operator or BT must explicitly clear.
- [x] Remove the 5-second hold timer and auto-clear logic from `publishUavState()`
- [x] Add `ClearEmergency.srv` to `peregrine_interfaces`
- [x] Implement the service in `uav_manager` (fires `EmergencyCleared` event through the FSM)
- [x] Ensure the `TransitionGuard` still enforces `EmergencyClearReady` conditions (guard unchanged, evaluated via applyEvent)

### 1.4 Abstract action server boilerplate
- **Problem:** Takeoff and Land action servers repeat identical scaffolding: lifecycle gate, emergency check, action slot reservation, RAII release guard, preempted/emergency lambdas.
- **Fix:** Create a `create_guarded_action_server<T>()` template or factory wrapper.
- **Note:** With only 2 action servers remaining after 1.1, the ROI is lower. Consider deferring unless more actions are added for the BT layer.
- [ ] Design the guarded action server interface (template on action type + orchestration callable)
- [ ] Extract common rejection logic (lifecycle, emergency, slot reservation) into the wrapper
- [ ] Migrate Takeoff and Land to use it

### 1.5 Template service polling helpers
- **Problem:** `callArmService()` and `callSetModeService()` (~90 LOC each) implement identical patterns: wait for service availability with 200ms poll + emergency check, then poll response future with 50ms intervals.
- **Fix:** `template<typename SrvT> StepResult pollService(client, request, deadline, emergencyCheck)`.
- [ ] Implement `pollService<T>()` template
- [ ] Migrate `callArmService()` and `callSetModeService()`

---

## Phase 2: C++ Codebase De-Sloping

### 2.1 ~~Nuke parameter boilerplate with `generate_parameter_library`~~
- **Status:** Moved to Phase 5.3 — do after BT layer is designed so the parameter surface is stable.

### 2.2 Fix the "wait for topic" blocking anti-pattern
- **Problem:** `on_configure()` blocks the lifecycle transition with `while` + `sleep_for(100ms)` waiting for PX4 topics. Appears in `control_manager`, `estimation_manager`, and `trajectory_manager`.
- **Fix:** Let `on_configure()` return `SUCCESS` instantly. Broadcast `WAITING_FOR_ESTIMATED_STATE` via status. Lock out flight operations until healthy.
- [x] Remove blocking loop from `control_manager::on_configure()`
- [x] Remove blocking loop from `estimation_manager::on_configure()`
- [x] Remove blocking loop from `trajectory_manager::on_configure()`
- [x] Verify that each manager's status publisher correctly reports "waiting" when data is absent
- [x] Verify that `uav_manager` health aggregator correctly gates on manager readiness
- [x] Remove `data_readiness_timeout_s` from `uav_manager` — now waits indefinitely for all dependencies

### 2.3 Lock-free telemetry caching
- **Problem:** Locking `std::mutex` 250x/second to copy small telemetry messages. `hardware_abstraction` already does this correctly with `std::atomic<std::shared_ptr<const Message>>`.
- **Fix:** Apply the same atomic shared_ptr pattern to `control_manager`, `trajectory_manager`, and `estimation_manager` subscriber callbacks.
- [x] Audit which subscriber callbacks currently use mutex for simple message caching
- [x] Convert to `std::atomic<std::shared_ptr<const T>>` with acquire/release ordering
- [x] Remove the now-unnecessary mutexes (keep mutexes where multi-field consistency is required)
- Note: `trajectory_manager` mutex retained — protects generators, goal handles, and goal type (complex state, not simple caching)

### 2.4 Extract lifecycle `stopPublishing()` helper
- **Problem:** `on_deactivate()` and `on_error()` are ~95% identical across `control_manager` and likely other lifecycle nodes (cancel timers, deactivate publishers, set active=false).
- **Fix:** Extract `void stopPublishing()` private method. Apply across all lifecycle nodes.
- [x] Extract in `control_manager`
- [x] Audit and extract in `trajectory_manager`
- [x] Audit and extract in `estimation_manager`

### 2.5 Split `onControlOutput()` in hardware_abstraction
- **Problem:** 202-line function handling 4 control modes with 60-110 lines per branch.
- **Fix:** Extract `handleTrajectoryMode()`, `handleBodyRateMode()`, `handleAttitudeMode()`, `handleDirectActuatorMode()`.
- [x] Extract the four mode handlers
- [x] Keep frame validation as a shared preamble

### 2.6 Fix HoldPositionGenerator lazy initialization
- **Package:** `trajectory_manager`
- **Problem:** Hold generator is null until first `estimated_state` arrives. If `on_activate()` fires first, there's a brief window with no hold generator.
- **Fix:** Create a default hold at origin in `on_configure()`. Overwrite when first state arrives.
- [x] Initialize `holdGenerator_` with safe default in `on_configure()`
- [ ] Document the fallback behavior

### 2.7 Eliminate virtual `name()` on trajectory generators
- **Problem:** 7 generators each override `name() const { return "hardcoded_string"; }` via virtual dispatch.
- **Fix:** Add `const std::string name_` member to base class, set in constructor. Remove virtual method.
- [x] Add `name_` member to `TrajectoryGeneratorBase`
- [x] Update all 7 generator constructors
- [x] Remove virtual `name()` override from each

### 2.8 Clean up dead includes
- [x] `generators.cpp`: remove `#include <limits>`
- [x] `generators.hpp`: remove `#include <cstdint>`
- [x] `rule_engine.cpp`: remove `#include <algorithm>`

---

## Phase 3: safety_monitor Improvements

### 3.1 Extract rule engine grace period logic
- **Problem:** `evaluateDetailed()` (67 lines) mixes checker invocation with grace period state tracking (5 fields per rule, 3 conditional branches per cycle).
- **Fix:** Extract `RuleState` class with `update(CheckResult, time_point)` and `isGraceExpired()`.
- [x] Create `RuleState` class
- [x] Migrate grace period logic out of `evaluateDetailed()`
- [ ] Add unit tests for grace period edge cases (deferred — logic is simple enough post-extraction)

### 3.2 Extract TUI health check handler pattern
- **Package:** `tui_status` — `tui_node.cpp`
- **Problem:** Three manager health callbacks (EST, CTL, TRJ) are structurally identical (~14 lines x3).
- **Fix:** Template helper or lambda factory parameterized by manager name.
- [x] Extract helper
- [x] Apply to all three manager status callbacks

---

## Phase 4: Launch & Python Overhaul

### 4.1 Create single `core_stack.launch.py` source of truth
- **Problem:** Manager container (4 composable nodes) is copy-pasted into 6+ launch files. Each example creates its own `ComposableNodeContainer`.
- **Fix:** One `core_stack.launch.py` in `peregrine_bringup`. All examples use `IncludeLaunchDescription`.
- [x] Create `core_stack.launch.py` with the full node composition
- [x] Accept YAML override paths as launch arguments
- [x] Migrate all example launch files to use `IncludeLaunchDescription`
- [x] Delete redundant container definitions (`peregrine_single_container.launch.py`, `example8_px4_sitl_single_uav.launch.py`)
- [x] Delete `single_uav_sitl.launch.py` — SITL is managed externally, not embedded in launch files
- [x] Update all references (`docker-compose.multi-sitl.yml`, `generate_multi_sitl.py`, `start_flight_stack.sh`) from `single_uav.launch.py` to `core_stack.launch.py`
- [x] Convert `peregrine_bringup` from `ament_cmake` to `ament_python`

### 4.2 Consolidate example launch files
- **Problem:** Separate SITL and non-SITL launch file variants for the same demo. SITL variants embed PX4/Gazebo startup that belongs in infrastructure, not examples.
- **Fix:** One launch file per demo, all include `core_stack.launch.py`. SITL management is external.
- [x] Delete SITL example variants (`se3_circle_figure8_sitl.launch.py`, `se3_step_response_sitl.launch.py`)
- [x] Delete dead tuning scripts (`se3_step_sweep.py`, `se3_circle_sweep.py`)
- [x] Rename examples to descriptive names (drop `exampleN_` prefix where done)
- [x] Update `examples/README.md`

### 4.3 YAML parameter config cleanup
- **Problem:** 18 config files in examples, many differing by 1 field or duplicating package defaults.
- **Fix:** Delete redundant configs, consolidate SE3 gains into `control_manager`, simplify example overrides.
- [x] Delete `example10_managers.yaml`, `example11_managers.yaml`, `example16_managers.yaml` (only set `status_rate_hz: 10.0`, package default of 5Hz is fine)
- [x] Delete `safety_diag_only.yaml`, `safety_land_enabled.yaml` (unreferenced)
- [x] Delete `circle_figure8_se3_mission.yaml`, `example10_se3_managers.yaml` (redundant with consolidated SE3 configs)
- [x] Move SE3 gains to `control_manager/config/se3_tuned.yaml` and `se3_conservative.yaml`
- [x] Rename `step_response_se3_mission.yaml` → `step_response_mission.yaml`
- [x] Update launch files: `circle_figure8_demo` and `multi_cycle_demo` use package defaults, controller switch demos load `se3_tuned.yaml` from `control_manager`
- [x] Retarget all demo script action clients from `uav_manager/*` to `trajectory_manager/*` (circle_figure8, controller_switch, controller_switch_inflight, multi_cycle, step_response, safety_regression)
- [x] Fix `step_response_demo.py` `step_sequence` parameter parsing (string-vs-list bug)
- [x] Update `controller_switch_demo.launch.py` and `controller_switch_inflight_demo.launch.py` to pass `uav_params_file` through to `core_stack`

### 4.4 Build `PeregrineClient` Python API
- **Problem:** Every demo script repeats: parameter declarations, UAVState subscription, action client setup, server wait logic, preflight readiness checks. ~60% of each script is identical boilerplate.
- **Fix:** `PeregrineClient` class encapsulating all ROS 2 client logic.
- [ ] Create `peregrine_client` Python package (or module in `peregrine_bringup`)
- [ ] Implement core methods: `arm()`, `takeoff(alt)`, `land()`, `execute_trajectory(type, params)`, `wait_ready()`
- [ ] Handle action client lifecycle internally (server wait, goal send, result polling)
- [ ] Provide async variants for BT integration
- [ ] Rewrite `circle_figure8_demo.py` using the client (proof of concept)
- [ ] Migrate all other demo scripts

### 4.5 Parameterize hardcoded paths
- [ ] Replace `/opt/PX4-Autopilot` with `$PX4_DIR` env var (default fallback to `/opt/PX4-Autopilot`) in all launch files
- [ ] Replace `/tmp/circle_eval.json` and similar with `$HOME/.ros/peregrine/` or parameterized paths

### 4.6 Parameter audit and cleanup
- **Problem:** Accumulated parameter surface had dead params, topic-name params that duplicate ROS 2 remapping, hardcoded physical constants exposed as tunables, MAVLink-era cargo params, and code-default/YAML-default drift.
- **Audit scope:** Every `declare_parameter` call across all C++ nodes, cross-referenced against `defaults.yaml` files and actual usage.
- [x] Remove dead `home_init_timeout_s` from `frame_transforms` (declared, stored, never read)
- [x] Remove dead `gps_freshness_timeout_s` from `frame_transforms` (declared, stored, never read)
- [x] Remove dead `dependency_startup_timeout_s` from `uav_manager` (only referenced in validation, never used for timeout logic — leftover from Phase 2.2 blocking-wait removal)
- [x] Remove topic name params from `safety_monitor` (`battery_topic`, `gps_status_topic`, `estimated_state_topic`, `px4_status_topic`, `map_frame`) — hardcoded defaults, use ROS 2 remapping if needed
- [x] Hardcode `se3.gravity` to 9.81 — physical constant, not a tunable
- [x] Fix `se3.mass` code default drift (was 1.5 in code vs 2.0643 in YAML)
- [x] Fix `se3.max_thrust_N` code default drift (was 29.43 in code vs 34.19432 in YAML)
- [x] Remove MAVLink `target_system_id`, `target_component_id`, `source_system_id`, `source_component_id` from `hardware_abstraction` — cargo from MAVLink era, DDS bridge handles routing, hardcoded to 1
- [x] Remove `home_init_timeout_s` from bringup YAML configs (`default.yaml`, `simulation.yaml`)
- [x] Remove `dependency_startup_timeout_s` from `simulation.yaml`
- [x] Remove `se3.gravity` from all SE3 YAML configs (`defaults.yaml`, `se3_tuned.yaml`, `se3_conservative.yaml`)
- **Deferred:** GPS quality threshold consolidation between `frame_transforms` and `safety_monitor` (same defaults, different nodes) — defer to Phase 5.3 (`generate_parameter_library`)
- **Deferred:** uav_manager polling timeout params (`service_wait_s`, `orchestrator_poll_ms`, etc.) — these are actively used in poll loops; the poll-loop architecture itself should be replaced with async ROS 2 patterns (Phase 1.5 or BT layer)

---

## Phase 5: Interface & Architectural Cleanup

### 5.1 ~~Audit GoTo action usage~~
- **Status:** Complete. GoTo is actively used — `trajectory_manager` serves the GoTo action directly, and `safety_regression_demo.py` uses it. `uav_manager` forwarding was removed in Phase 1.1. `acceptance_radius_m` was removed from the action definition; feedback changed from `distance_remaining_m` to `progress`.
- [x] Grep for GoTo usage across the codebase
- [x] GoTo is used by `trajectory_manager` (serves directly) and `safety_regression_demo.py`
- [x] Removed `acceptance_radius_m` field and updated feedback (Phase 1.1)

### 5.2 Evaluate custom vs standard ROS messages
- **Problem:** `GpsStatus.msg` fields overlap with `sensor_msgs/NavSatFix`. `PX4Status.msg` has PX4-specific fields (nav_state, arming_state) that don't map to standard messages.
- [ ] Document why `PX4Status.msg` is custom (PX4-specific enum fields — likely justified)
- [ ] Evaluate if `GpsStatus.msg` can be replaced with `sensor_msgs/NavSatFix` + a small supplement
- [ ] If custom messages are kept, add comments explaining why standard alternatives were rejected

### 5.3 Migrate to `generate_parameter_library`
- **Problem:** Every node manually calls `declare_parameter` + `get_parameter` with hand-written defaults and validation. Drift between code defaults and YAML defaults is easy.
- **Fix:** Define parameter schemas in YAML, generate type-safe C++ structs at build time. Single source of truth for names, types, defaults, ranges.
- **Do after BT layer (Phase 7) is designed** — the BT will change what parameters each node needs.
- [ ] Add `generate_parameter_library` build dependency
- [ ] Convert `safety_monitor` parameters (highest ROI — 39 declarations)
- [ ] Convert `uav_manager` parameters
- [ ] Convert `control_manager` parameters
- [ ] Convert `trajectory_manager` parameters
- [ ] Convert `estimation_manager` parameters
- [ ] Convert `hardware_abstraction` parameters
- [ ] Delete per-package `defaults.yaml` — schema files become the source of truth

### 5.4 Add missing test coverage
- [ ] `frame_transforms`: add `geodeticToEnu()` test with known GPS coordinates
- [ ] `trajectory_manager`: add generator progress tests (verify `progress` reaches 1.0 at expected time for each generator type)
- [ ] `safety_monitor`: add rule engine grace period unit tests (per Phase 3)

### 5.5 Document launch architecture decisions
- [ ] Add a decision matrix: when to use single-container vs two-container vs three-container
- [ ] Document the temporal hierarchy (hard RT, soft RT, reflexes, BT) in an ARCHITECTURE.md
- [ ] Document the FSM vs BT separation of concerns

---

## Phase 6: Production Readiness

### 6.1 Kill Tmuxinator for hardware flight
- **Problem:** `docker/config/tmuxinator/flight.yml` uses tmux panes to run ROS 2 nodes. Crashed nodes leave dead terminals, break lifecycle tracking, and make logging unreliable.
- **Fix for development (SITL):** Keep tmuxinator, it's fine for developers.
- **Fix for hardware (Jetson/RPi5):** Docker entrypoint runs exactly one command: `ros2 launch peregrine_bringup hardware_mission.launch.py`. ROS 2 launch manages processes, restarts, and logging.
- [ ] Create `hardware_mission.launch.py` (or verify it exists and is complete)
- [ ] Update Docker entrypoint for hardware to use `ros2 launch` instead of tmuxinator
- [ ] Verify all nodes are included in the hardware launch file
- [ ] Test process restart behavior (node crash → launch system restarts it)
- [ ] Verify logs route to ROS `/log` directory and Docker daemon

---

## Phase 7: BT Package Scaffolding & ROS 2 Bridge

The BT becomes the "pilot" — it reads UAV state, decides intent, and ticks actions. It does NOT hold flight state (the FSM in `uav_manager` does that). The integration is split across Phases 7-10 to make scope and dependencies explicit.

**Prerequisite:** Phases 0-1 must be complete. Phase 2 strongly recommended (removes blocking anti-patterns that would interfere with BT tick timing).

### 7.1 Create the `peregrine_bt` package
- **Library:** [BehaviorTree.CPP v4](https://github.com/BehaviorTree/BehaviorTree.CPP) — the de facto standard for ROS 2 BT applications.
- [ ] Add `behaviortree_cpp` as a dependency (available via `apt` on most ROS 2 distros)
- [ ] Create `peregrine_bt` package with `CMakeLists.txt`, `package.xml`
- [ ] Verify BT.CPP v4 builds against current ROS 2 distribution (Humble)
- [ ] Add package to `core_stack.launch.py` as an optional composable node (disabled by default via launch argument)
- [ ] Verify the package builds in the Docker simulation image

### 7.2 Design the BT-ROS 2 bridge layer
The BT needs thin wrappers that map BT action/condition nodes to ROS 2 clients. These must be generic and reusable — mission-specific logic belongs in the tree XML, not in bridge code.

#### 7.2.1 `RosActionNode<T>` base template
- [ ] Wraps `rclcpp_action::Client<T>`, handles goal send/cancel/result as BT RUNNING/SUCCESS/FAILURE
- [ ] On `onStart()`: send goal asynchronously, return RUNNING
- [ ] On `onRunning()`: check goal handle status — RUNNING if still executing, SUCCESS/FAILURE on result
- [ ] On `onHalted()`: cancel the goal if still active (BT tree pre-emption)
- [ ] Timeout: configurable per-node via BT port, returns FAILURE on expiry
- [ ] Error mapping: goal rejected → FAILURE, goal aborted → FAILURE with error code on blackboard

#### 7.2.2 `RosServiceNode<T>` base template
- [ ] Wraps `rclcpp::Client<T>` for synchronous-style service calls
- [ ] On `onStart()`: send request asynchronously, return RUNNING
- [ ] On `onRunning()`: poll future — SUCCESS when response arrives, FAILURE on timeout or service error
- [ ] On `onHalted()`: no cancel needed (services are fire-and-forget), just stop polling
- [ ] Service availability: FAILURE immediately if service is not available (no blocking wait)

#### 7.2.3 `RosTopicCondition<T>` base template
- [ ] Subscribes to a topic, caches latest message via atomic shared_ptr (same pattern as existing telemetry caching)
- [ ] Exposes latest value to BT blackboard on each tick
- [ ] Staleness check: configurable max age — if latest message is older than threshold, condition returns FAILURE
- [ ] Subscription is created once at node construction, not per-tick

#### 7.2.4 Blackboard conventions
- [ ] Define standard blackboard key names: `uav_state`, `safety_status`, `estimated_state`, `battery_state`, `gps_status`
- [ ] Define typed port conventions: input ports for parameters (altitude, radius), output ports for results (error codes, final positions)
- [ ] Document thread safety: blackboard writes only from subscriber callbacks or tick thread, never both for the same key

### 7.3 Groot2 visualization setup
- [ ] Verify Groot2 compatibility with BT.CPP v4 version
- [ ] Create a `groot2_publisher` integration in the BT executor node (ZMQ-based tree status broadcast)
- [ ] Document how to connect Groot2 to a running BT for live visualization
- [ ] Create a convenience launch argument (`enable_groot:=true`) to activate the publisher

---

## Phase 8: BT Node Implementation

Each BT node is a thin adapter between the tree and a specific ROS 2 interface. No business logic — if a node needs more than ~30 lines beyond the base template, the complexity likely belongs in the ROS 2 server, not the BT node.

### 8.1 Condition nodes (pure readers)
These check world state and return SUCCESS/FAILURE. They do NOT send commands or modify state.

#### 8.1.1 UAV state conditions
- [ ] `IsArmed` — reads `UAVState.armed`
- [ ] `IsFlying` — reads `UAVState.supervisor_state == FLYING`
- [ ] `IsLanded` — reads `UAVState.supervisor_state == LANDED_DISARMED` or `LANDED_ARMED`
- [ ] `IsDependenciesReady` — reads `UAVState.dependencies_ready`
- [ ] `HasValidState` — checks `estimated_state` freshness (message age < threshold)

#### 8.1.2 Safety conditions
- [ ] `IsSafetyNominal` — reads `SafetyStatus.overall_level == NOMINAL`
- [ ] `IsSafetyAtLeast(level)` — parameterized: returns SUCCESS if safety level >= input port value (allows trees to tolerate WARN but not CRITICAL)

#### 8.1.3 Sensor conditions
- [ ] `IsBatteryAbove(threshold_pct)` — reads battery percentage against input port threshold
- [ ] `IsGpsHealthy` — reads GPS fix type >= 3D fix
- [ ] `IsGpsHdopBelow(threshold)` — reads GPS HDOP against input port threshold

#### 8.1.4 Position conditions
- [ ] `IsAtPosition(target, tolerance_m)` — compares current position to blackboard target within tolerance
- [ ] `IsAboveAltitude(min_alt_m)` — checks current altitude against threshold
- [ ] `IsBelowAltitude(max_alt_m)` — checks current altitude against ceiling

### 8.2 Action nodes (effectors)
These call ROS 2 actions/services and report progress. All must be async (return RUNNING, not block).

#### 8.2.1 Flight lifecycle actions
- [ ] `ArmAction` — calls `uav_manager/arm` service via `RosServiceNode<Arm>`
- [ ] `TakeoffAction(altitude_m, climb_velocity_mps)` — calls `uav_manager/takeoff` action via `RosActionNode<Takeoff>`, exposes feedback (current altitude) on blackboard
- [ ] `LandAction(descent_velocity_mps)` — calls `uav_manager/land` action via `RosActionNode<Land>`, exposes feedback (altitude remaining) on blackboard

#### 8.2.2 Trajectory actions
- [ ] `ExecuteTrajectoryAction(type, params...)` — calls `trajectory_manager/execute_trajectory` directly (NOT through uav_manager — that forwarding was removed in Phase 1)
- [ ] Input ports: `trajectory_type` (string), plus type-specific ports (`radius`, `angular_velocity`, `loops`, `step_size`, etc.)
- [ ] Output ports: `completion_error_m` (final position error at trajectory end)
- [ ] Feedback: expose `progress_pct` on blackboard for monitoring

#### 8.2.3 Mode and recovery actions
- [ ] `SetModeAction(mode)` — calls `hardware_abstraction/set_mode` service
- [ ] `ClearEmergencyAction` — calls `uav_manager/clear_emergency` service (added in Phase 1.3)

#### 8.2.4 Utility actions
- [ ] `WaitAction(duration_s)` — simple timer-based wait, returns RUNNING until duration elapsed, uses ROS clock (not wall clock) for sim-time compatibility
- [ ] `LogAction(message, level)` — publishes a RCLCPP log message at configurable severity (useful for debugging trees)
- [ ] `SetBlackboardAction(key, value)` — writes a value to the blackboard (useful for parameterizing subtrees)

### 8.3 BT node registration and plugin discovery
- [ ] Register all nodes with the `BT::BehaviorTreeFactory` using `registerNodeType<>()`
- [ ] Organize registrations in a single `register_nodes.cpp` file so new nodes are easy to find
- [ ] Add `BT_REGISTER_NODES` macro export for potential plugin-based loading (future: user-defined nodes without recompiling `peregrine_bt`)
- [ ] Verify all nodes appear in Groot2's node palette

---

## Phase 9: Mission Trees & BT Executor

### 9.1 BT executor node (`PeregrineBTNode`)
The executor is a ROS 2 lifecycle node that loads a tree XML, ticks it, and publishes status. It owns the `rclcpp::Node` that all BT nodes share for subscriptions and clients.

- [ ] Create `PeregrineBTNode` as a lifecycle node
- [ ] Parameter: `tree_file` (string) — path to the XML tree to execute
- [ ] Parameter: `tick_rate_hz` (double, default 2.0) — BT tick frequency
- [ ] On `on_configure()`: load XML, register all node types, create the tree
- [ ] On `on_activate()`: start the tick timer
- [ ] On `on_deactivate()`: halt the tree (calls `onHalted()` on all active action nodes), cancel tick timer
- [ ] Publish tree status on `bt_status` topic each tick (current state of root, number of RUNNING nodes, active action names)
- [ ] Graceful shutdown: on deactivation or SIGINT, halt active actions before destroying tree

### 9.2 Shared ROS node design
BT.CPP nodes need access to a `rclcpp::Node` for creating subscriptions and clients. The design must avoid creating hundreds of nodes.

- [ ] Single shared `rclcpp::Node` (or the executor node itself) passed to all BT nodes via the blackboard
- [ ] All subscriptions created at tree creation time, not per-tick
- [ ] All action/service clients created at tree creation time with lazy connection (connect on first use)
- [ ] Document: the shared node runs in the executor's callback group — BT nodes must not block callbacks

### 9.3 Core mission trees (XML)
Start with simple trees that replicate existing demo scripts, then build up complexity. Each tree file lives in `peregrine_bt/trees/`.

#### Tree 1: `takeoff_hover_land.xml` — Smoke test
```
<BehaviorTree ID="TakeoffHoverLand">
  <Sequence>
    <IsDependenciesReady/>
    <ArmAction/>
    <TakeoffAction altitude_m="5.0" climb_velocity_mps="1.0"/>
    <WaitAction duration_s="10.0"/>
    <LandAction descent_velocity_mps="0.8"/>
  </Sequence>
</BehaviorTree>
```
- [ ] Implement and validate in SITL
- [ ] Verify arm, takeoff, hover, land sequence completes without error
- [ ] Verify BT status topic shows correct state transitions

#### Tree 2: `circle_with_safety.xml` — Reactive safety fallback
```
<BehaviorTree ID="CircleWithSafety">
  <ReactiveFallback>
    <ReactiveSequence>
      <IsSafetyNominal/>
      <Sequence>
        <IsDependenciesReady/>
        <ArmAction/>
        <TakeoffAction altitude_m="5.0" climb_velocity_mps="1.0"/>
        <ExecuteTrajectoryAction trajectory_type="circle" radius="2.0"
                                 angular_velocity="0.6" loops="2"/>
        <LandAction descent_velocity_mps="0.8"/>
      </Sequence>
    </ReactiveSequence>
    <Sequence name="SafetyLand">
      <LogAction message="Safety degraded — emergency land" level="WARN"/>
      <LandAction descent_velocity_mps="1.0"/>
    </Sequence>
  </ReactiveFallback>
</BehaviorTree>
```
- [ ] Implement and validate in SITL
- [ ] Test: trigger a geofence warning mid-circle, verify BT switches to safety land
- [ ] Test: verify normal completion when no safety events occur

#### Tree 3: `multi_trajectory.xml` — Circle + figure-8 with retry
```
<BehaviorTree ID="MultiTrajectory">
  <Sequence>
    <SubTree ID="ArmAndTakeoff" altitude_m="5.0"/>
    <RetryNode num_attempts="3">
      <ExecuteTrajectoryAction trajectory_type="circle" radius="2.0"
                               angular_velocity="0.6" loops="1"/>
    </RetryNode>
    <RetryNode num_attempts="3">
      <ExecuteTrajectoryAction trajectory_type="figure8" radius="2.0"
                               angular_velocity="0.6" loops="1"/>
    </RetryNode>
    <LandAction descent_velocity_mps="0.8"/>
  </Sequence>
</BehaviorTree>
```
- [ ] Implement with a reusable `ArmAndTakeoff` subtree
- [ ] Validate in SITL — verify retry on trajectory failure
- [ ] Verify that a failed trajectory followed by a retry does not leave stale state on the blackboard

#### Tree 4: `controller_switch_demo.xml` — Replace controller_switch_demo.py
- [ ] Replicate the existing controller switch demo as a BT tree
- [ ] Sequence: takeoff → switch to SE3 → circle → switch back to passthrough → land
- [ ] Verify controller switch mid-flight works correctly through BT action nodes

### 9.4 Subtree library
Common patterns extracted into reusable subtrees that missions compose via `<SubTree>`.
- [ ] `ArmAndTakeoff` — IsDependenciesReady → Arm → Takeoff (parameterized altitude)
- [ ] `SafetyLand` — LogAction → LandAction (used as fallback in reactive trees)
- [ ] `PreflightChecks` — IsDependenciesReady → IsBatteryAbove → IsGpsHealthy → IsSafetyNominal
- [ ] Store subtrees in `peregrine_bt/trees/subtrees/`
- [ ] Document how to compose subtrees into custom missions

### 9.5 Launch integration
- [ ] Add `enable_bt` launch argument to `core_stack.launch.py` (default: false)
- [ ] When enabled, launch `PeregrineBTNode` as a composable node in the manager container
- [ ] Add `tree_file` and `tick_rate_hz` as launch arguments
- [ ] Create `bt_mission.launch.py` convenience launcher: includes `core_stack.launch.py` with `enable_bt:=true` and accepts a tree file path

---

## Phase 10: BT Testing & Operational Readiness

### 10.1 Temporal hierarchy compliance
The BT operates at the **slowest layer** (1-10Hz). It must NOT:
- Bypass the FSM by sending raw PX4 commands
- Hold references to high-frequency data (subscribe, snapshot, release)
- Block on long computations during tick (offload to async action nodes)
- Assume tick timing is deterministic (use timeouts, not tick counts)

- [ ] All action nodes verified async (return RUNNING, never block tick thread)
- [ ] All condition nodes verified non-blocking (read cached blackboard values only)
- [ ] Blackboard values updated by dedicated subscriber callbacks, NOT during tick
- [ ] Document the tick rate contract: "BT is safe to stutter or pause without affecting flight safety — the FSM and safety_monitor handle real-time concerns"
- [ ] Measure worst-case tick duration under load — must stay under 50ms at 2Hz tick rate

### 10.2 Unit tests
- [ ] Test each condition node in isolation: mock ROS topic data on blackboard, verify SUCCESS/FAILURE thresholds
- [ ] Test each action node in isolation: mock ROS action/service server, verify RUNNING→SUCCESS and RUNNING→FAILURE paths
- [ ] Test `RosActionNode<T>` base: verify goal cancellation on `onHalted()`, timeout behavior, error code propagation
- [ ] Test `RosServiceNode<T>` base: verify service-unavailable returns FAILURE (no hang), timeout behavior
- [ ] Test blackboard staleness: verify condition nodes return FAILURE when cached data is older than threshold

### 10.3 Integration tests (SITL)
- [ ] Full stack smoke test: SITL + core_stack + BT executor, run Tree 1, verify takeoff/hover/land
- [ ] Safety fallback test: run Tree 2, inject geofence violation mid-flight, verify reactive land
- [ ] Retry test: run Tree 3, kill trajectory_manager briefly, verify retry succeeds after restart
- [ ] Controller switch test: run Tree 4, verify SE3 ↔ passthrough switching in flight
- [ ] Multi-UAV test: two UAVs each running independent BT trees, verify no cross-talk

### 10.4 Failure mode tests
- [ ] Kill `safety_monitor` mid-flight — verify BT's reactive fallback triggers land (safety status goes stale → IsSafetyNominal returns FAILURE)
- [ ] Kill `trajectory_manager` mid-trajectory — verify action node returns FAILURE, tree handles it (retry or land)
- [ ] Kill `uav_manager` mid-takeoff — verify BT detects loss of UAV state, triggers fallback
- [ ] Simulate PX4 disconnect (stop the SITL instance) — verify stack degrades gracefully, BT does not hang on RUNNING forever
- [ ] Tick starvation: artificially slow the BT tick rate to 0.5Hz, verify flight safety is unaffected (FSM and safety_monitor handle RT concerns independently)

### 10.5 Observability and debugging
- [ ] Groot2 integration verified: connect to running BT, visualize live tree state
- [ ] Groot2 recording: capture tree execution traces for post-flight analysis (log to file)
- [ ] TUI integration: add BT status panel to `tui_status` showing current tree state, active action, tick rate
- [ ] `bt_status` topic documented: message format, update rate, how to interpret states

### 10.6 Demo script migration
Once BT trees replicate existing demo behavior, migrate demos from Python scripts to BT XML + launcher.
- [ ] `circle_figure8_demo.py` → `circle_figure8.xml` + `bt_mission.launch.py tree_file:=...`
- [ ] `multi_cycle_demo.py` → `multi_trajectory.xml`
- [ ] `controller_switch_demo.py` → `controller_switch_demo.xml`
- [ ] `controller_switch_inflight_demo.py` → `controller_switch_inflight.xml`
- [ ] `step_response_demo.py` → `step_response.xml`
- [ ] Keep Python scripts as reference/fallback until BT trees are validated in SITL
- [ ] Delete Python demo scripts after BT equivalents are validated (or mark deprecated)

### 10.7 Documentation
- [ ] `peregrine_bt/README.md`: package overview, how to write a custom tree, how to add new BT nodes
- [ ] Document the FSM vs BT separation of concerns: what the FSM owns (flight state, safety gates) vs what the BT owns (mission sequencing, recovery logic)
- [ ] Document the temporal hierarchy (hard RT at 250Hz → soft RT at 50Hz → reflexes at 2Hz → BT at 1-10Hz)
- [ ] Add example: "How to create a new mission" walkthrough (copy a tree XML, customize parameters, launch)

---

## Summary: Effort Estimates

| Phase | Estimated Effort | Priority |
|-------|-----------------|----------|
| Phase 0: Safety fixes | 2-3 days | **Immediate** |
| Phase 1: uav_manager strip-down | 3-4 days | High |
| Phase 2: C++ de-sloping | 1-2 weeks | High |
| Phase 3: safety_monitor improvements | 2-3 days | Medium |
| Phase 4: Launch & Python overhaul | 1-2 weeks | Medium |
| Phase 5: Interface & architecture cleanup | 3-4 days | Medium |
| Phase 6: Production readiness | 2-3 days | Before hardware flight |
| Phase 7: BT package scaffolding & bridge | 1-2 weeks | After Phases 0-2 |
| Phase 8: BT node implementation | 1-2 weeks | After Phase 7 |
| Phase 9: Mission trees & executor | 1-2 weeks | After Phase 8 |
| Phase 10: BT testing & operational readiness | 1-2 weeks | After Phase 9 |
