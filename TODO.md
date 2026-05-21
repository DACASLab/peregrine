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
- **Status:** Complete — see Phase 5.3. All 7 nodes converted to `generate_parameter_library`.

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
- [x] Create `peregrine_client` Python package
- [x] Implement core methods: `arm()`, `takeoff(alt)`, `land()`, `execute()`, `go_to()`, `wait_ready()`, `clear_emergency()`, `set_mode()`
- [x] Handle action client lifecycle internally (server wait, goal send, result polling)
- [ ] Provide async variants for BT integration
- [x] Rewrite `circle_figure8_demo.py` using the client (proof of concept)
- [x] Migrate `multi_cycle_demo.py`, `controller_switch_demo.py`, `controller_switch_inflight_demo.py`, `step_response_demo.py`
- [ ] Migrate remaining safety test scripts (`safety_regression_demo`, `safety_takeoff_hold_demo`, `safety_fault_injector`)

### 4.5 Parameterize hardcoded paths
- [x] Replace `/opt/PX4-Autopilot` with `px4_autopilot_dir` launch argument (default `/opt/PX4-Autopilot`) in `multi_uav_sitl.launch.py`
- [x] `/tmp/circle_eval.json` and `/tmp/step_response_eval.json` are already ROS parameters (`output_path`) — overridable via `--ros-args`

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
- [x] Remove MAVLink `target_component_id`, `source_system_id`, `source_component_id` from `hardware_abstraction` — cargo from MAVLink era, hardcoded to 1
- [x] Re-add `target_system_id` as a parameter (default 1) — PX4 Commander validates `target_system` against `MAV_SYS_ID` even over uXRCE-DDS; multi-instance SITL sets `MAV_SYS_ID = instance+1`, so hardcoding to 1 silently rejects commands for instances > 0
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

### 5.2 ~~Evaluate custom vs standard ROS messages~~
- **Status:** Evaluated — both custom messages are justified and kept.
- [x] `PX4Status.msg` — consolidates PX4-specific fields (`nav_state`, `arming_state`, `failure_detector_status`, `motor_output`) from multiple px4_msgs into one message. No standard ROS equivalent exists; replacing would leak `px4_msgs` into the stack or require multiple messages.
- [x] `GpsStatus.msg` — `sensor_msgs/NavSatFix` lacks `hdop`, `vdop`, `eph`, `epv`, `satellites_used` which `safety_monitor` and `frame_transforms` need for GPS health gating. Replacing would require NavSatFix + a supplementary message for no benefit.
- [x] Both are thin translation layers published by `hardware_abstraction`, consumed by `safety_monitor`, `tui_status`, `frame_transforms`. Custom messages are justified.

### 5.3 ~~Migrate to `generate_parameter_library`~~
- **Status:** Complete. All 7 nodes converted. Parameter YAML schemas are now the single source of truth. Clean Docker build verified (15/15 packages, 0 errors).
- [x] Add `generate_parameter_library` to all Dockerfiles (`ros-${ROS_DISTRO}-generate-parameter-library`)
- [x] Convert `safety_monitor` parameters (33 params)
- [x] Convert `uav_manager` parameters (9 params)
- [x] Convert `control_manager` parameters (5 node-level params; SE3 controller uses `declareOrGet` plugin pattern, stays manual)
- [x] Convert `trajectory_manager` parameters (4 params)
- [x] Convert `estimation_manager` parameters (4 params)
- [x] Convert `hardware_abstraction` parameters (7 params)
- [x] Convert `frame_transforms` parameters (14 params)
- [ ] Delete per-package `defaults.yaml` — schema files become the source of truth

### 5.4 Add missing test coverage
- [ ] `frame_transforms`: add `geodeticToEnu()` test with known GPS coordinates
- [ ] `trajectory_manager`: add generator progress tests (verify `progress` reaches 1.0 at expected time for each generator type)
- [ ] `safety_monitor`: add rule engine grace period unit tests (per Phase 3)

### 5.5 Document launch architecture decisions
- [ ] Add a decision matrix: when to use single-container vs two-container vs three-container
- [x] Document the temporal hierarchy (hard RT, soft RT, reflexes, BT) in `docs/ARCHITECTURE.md`
- [x] Document the FSM vs BT separation of concerns in `docs/ARCHITECTURE.md`

---

## Phase 6: Production Readiness

### 6.1 Kill Tmuxinator for hardware flight
- **Problem:** `docker/config/tmuxinator/flight.yml` uses tmux panes to run ROS 2 nodes. Crashed nodes leave dead terminals, break lifecycle tracking, and make logging unreliable. TUI on hardware wastes resources — monitoring belongs on GCS.
- **Fix for development (SITL):** Keep tmuxinator (`peregrine.yml`), it's fine for developers.
- **Fix for hardware (Jetson/RPi5):** Container runs `start_flight_stack.sh` directly → `ros2 launch core_stack.launch.py`. Flight stack is PID 1 — Docker handles restart on crash. Debugging via `make shell-jetson` / `make shell-rpi5`.
- [x] Delete `docker/config/tmuxinator/flight.yml`
- [x] Update Jetson/RPi5 docker-compose `command` to run `start_flight_stack.sh` directly
- [x] Remove `ruby-full` and `gem install tmuxinator` from Jetson/RPi5 Dockerfiles
- [x] Remove tmuxinator config COPY/setup from Jetson/RPi5 Dockerfiles
- [x] TUI stays on GCS only (already in `gcs.generated.yml`)
- [x] Logs: ROS 2 `~/.ros/log/` + `docker logs` — no tmux scrollback needed
- [x] Hardware debugging: `make shell-jetson` / `make shell-rpi5` → `docker exec aircraft bash`

---

## Phase 7: BT Package Scaffolding & ROS 2 Bridge

The BT becomes the "pilot" — it reads UAV state, decides intent, and ticks actions. It does NOT hold flight state (the FSM in `uav_manager` does that). The integration is split across Phases 7-10 to make scope and dependencies explicit.

**Prerequisite:** Phases 0-1 must be complete. Phase 2 strongly recommended (removes blocking anti-patterns that would interfere with BT tick timing).

### 7.1 ~~Create the `peregrine_bt` package~~
- **Status:** Complete. Package created with BT.CPP v4 + official `behaviortree_ros2` bridge.
- **Library:** [BehaviorTree.CPP v4](https://github.com/BehaviorTree/BehaviorTree.CPP) — the de facto standard for ROS 2 BT applications.
- **Bridge:** [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) — official ROS 2 integration by the BT.CPP author. Provides `RosActionNode<T>`, `RosServiceNode<T>`, `RosTopicSubNode<T>`, `RosTopicPubNode<T>` base templates. Not available via apt for Humble — cloned as a git submodule in `src/behaviortree_ros2`.
- **Process isolation:** The BT executor runs as a **separate process**, not a composable node in the core stack container. See `docs/ARCHITECTURE.md` for rationale. Launch integration comes in Phase 9.5 via `bt_mission.launch.py`.
- [x] Add `behaviortree_cpp` as a dependency (available via `apt` on Humble)
- [x] Add `behaviortree_ros2` as a git submodule in `src/` (not on apt for Humble)
- [x] Create `peregrine_bt` package with `CMakeLists.txt`, `package.xml`
- [x] Delete empty `mission_executor` skeleton (replaced by `peregrine_bt`)
- [x] Verify BT.CPP v4 (4.9.0) builds against ROS 2 Humble
- [x] Verify the package builds in the Docker simulation image (19 packages, 0 errors)

### 7.2 ~~BT-ROS 2 bridge layer~~
- **Status:** Superseded — using the official `behaviortree_ros2` package instead of custom templates. The official library provides `RosActionNode<T>`, `RosServiceNode<T>`, `RosTopicSubNode<T>` with client reuse via static registry, proper error enums, callback group isolation, and non-blocking async execution.
- [x] `RosActionNode<T>` — provided by `behaviortree_ros2`
- [x] `RosServiceNode<T>` — provided by `behaviortree_ros2`
- [x] `RosTopicSubNode<T>` — provided by `behaviortree_ros2`
- [x] Blackboard conventions — not needed; the official library uses typed input/output ports per node and `default_port_value` in `RosNodeParams` for topic/action/service names

### 7.3 Groot2 visualization setup
- **Deferred** until mission trees are implemented and testable in SITL (Phase 9/10). The `behaviortree_ros2` `TreeExecutionServer` has built-in Groot2 publisher support.
- [ ] Verify Groot2 compatibility with BT.CPP v4 version
- [ ] Enable Groot2 publisher in the BT executor node
- [ ] Document how to connect Groot2 to a running BT for live visualization

---

## Phase 8: BT Node Implementation

Each BT node is a thin adapter between the tree and a specific ROS 2 interface. No business logic — if a node needs more than ~30 lines beyond the base template, the complexity likely belongs in the ROS 2 server, not the BT node. All nodes use the official `behaviortree_ros2` base templates.

### 8.1 ~~Condition nodes (pure readers)~~
- **Status:** Complete. 15 condition nodes implemented as `RosTopicSubNode<T>` subclasses.

#### 8.1.1 ~~UAV state conditions~~ (subscribe to `uav_state`)
- [x] `IsArmed` — reads `UAVState.armed`
- [x] `IsFlying` — reads `UAVState.state == FLYING` or `HOVERING`
- [x] `IsLanded` — reads `UAVState.state == IDLE` or `LANDED`
- [x] `IsDependenciesReady` — reads `UAVState.dependencies_ready`
- [x] `IsEmergency` — reads `UAVState.state == EMERGENCY` (added — needed for reactive tree guards)
- [x] `IsConnected` — reads `UAVState.connected` (added — needed to detect PX4 disconnect)
- [x] `IsOffboard` — reads `UAVState.offboard` (added — useful pre-trajectory check)

#### 8.1.2 ~~Safety conditions~~ (subscribe to `safety_status`)
- [x] `IsSafetyNominal` — reads `SafetyStatus.level == NOMINAL`
- [x] `IsSafetyAtLeast(max_level)` — returns SUCCESS if `level <= max_level` input port

#### 8.1.3 ~~Sensor conditions~~
- [x] `IsBatteryAbove(threshold_pct)` — reads `PX4Status.battery_remaining` (subscribes to `status`)
- [x] `IsGpsHealthy(min_fix_type, max_hdop, max_vdop)` — reads fix type, HDOP, and VDOP (subscribes to `gps_status`). Merged `IsGpsHdopBelow` — separate node was unnecessary.

#### 8.1.4 ~~Position conditions~~ (subscribe to `state` — estimated state)
- [x] `HasValidState(max_age_s)` — checks estimated state freshness via header timestamp
- [x] `IsAtPosition(target_x, target_y, target_z, tolerance_m)` — 3D Euclidean distance check
- [x] `IsAboveAltitude(min_alt_m)` — checks `pose.position.z`
- [x] `IsBelowAltitude(max_alt_m)` — checks `pose.position.z`

### 8.2 ~~Action and service nodes (effectors)~~
- **Status:** Complete. 4 action nodes, 3 service nodes, 1 utility node.

#### 8.2.1 ~~Flight lifecycle~~
- [x] `ArmService` — calls `arm` service via `RosServiceNode<Arm>` (on `hardware_abstraction`)
- [x] `TakeoffAction(altitude_m, climb_velocity_mps)` — calls `uav_manager/takeoff` action, output port `final_altitude_m`
- [x] `LandAction(descent_velocity_mps)` — calls `uav_manager/land` action

#### 8.2.2 ~~Trajectory actions~~
- [x] `ExecuteTrajectoryAction(trajectory_type, params)` — calls `trajectory_manager/execute_trajectory`. Params passed as comma-separated string, parsed to `float64[]`.
- [x] `GoToAction(x, y, z, yaw, velocity_mps)` — calls `trajectory_manager/go_to` (added — GoTo is an actively used action, was missing from original TODO)

#### 8.2.3 ~~Mode and recovery~~
- [x] `SetModeService(mode)` — calls `set_mode` service (on `hardware_abstraction`)
- [x] `ClearEmergencyService` — calls `uav_manager/clear_emergency` service

#### 8.2.4 ~~Utility~~
- [x] `WaitAction(duration_s)` — `StatefulActionNode` with ROS clock deadline (sim-time compatible)
- ~~`LogAction`~~ — dropped; BT.CPP v4 built-in `StdCoutLogger` and script nodes cover this
- ~~`SetBlackboardAction`~~ — dropped; BT.CPP v4 has a built-in `SetBlackboard` node

### 8.3 ~~BT node registration~~
- **Status:** Complete. All 23 nodes registered in `register_nodes.cpp` with default topic/action/service names via `RosNodeParams`.
- [x] All nodes registered with `BT::BehaviorTreeFactory` using `registerNodeType<>()`
- [x] Registrations organized in `register_nodes.cpp` with default `RosNodeParams` per node
- [ ] Add `BT_REGISTER_NODES` macro export for plugin-based loading (deferred — not needed until user-defined BT nodes)

---

## Phase 9: Mission Trees & BT Executor

### 9.1 ~~BT executor node (`BTExecutorNode`)~~
- **Status:** Complete. Lifecycle node with tick timer, shared client node, and `MultiThreadedExecutor`.
- [x] Create `BTExecutorNode` as a lifecycle node
- [x] Parameter: `tree_file` (string) — path to the XML tree to execute
- [x] Parameter: `tick_rate_hz` (double, default 2.0) — BT tick frequency
- [x] On `on_configure()`: load XML, register all node types via `registerAllNodes()`, create tree
- [x] On `on_activate()`: start the tick timer
- [x] On `on_deactivate()`: halt the tree, cancel tick timer
- [x] Graceful shutdown: `on_shutdown()` halts tree and cleans up
- [ ] Publish tree status on `bt_status` topic each tick (current state of root, number of RUNNING nodes, active action names)

### 9.2 ~~Shared ROS node design~~
- **Status:** Complete. Separate `rclcpp::Node` ("bt_client") created in the executor constructor, shared to all BT nodes via `RosNodeParams`. Both nodes added to a `MultiThreadedExecutor`.
- [x] Single shared `rclcpp::Node` passed to all BT nodes via `RosNodeParams`
- [x] The `behaviortree_ros2` library handles subscription/client reuse via static registries — multiple nodes subscribing to the same topic share one subscription
- [x] `MultiThreadedExecutor` spins both the lifecycle node and the client node
- [x] Forward `use_sim_time` from the lifecycle node to the client node — required for correct `node->now()` in SITL (affects `HasValidState` timestamp comparison and `WaitAction` deadline)

### 9.3 Core mission trees (XML)
Start with simple trees that replicate existing demo scripts, then build up complexity. Each tree file lives in `peregrine_bt/trees/`.

#### ~~Tree 1: `takeoff_hover_land.xml` — Smoke test~~
- **Status:** Complete. Sequence: IsDependenciesReady → TakeoffAction → WaitAction 10s → LandAction.
- [x] Tree XML implemented
- [ ] Validate in SITL
- [ ] Verify BT status topic shows correct state transitions

#### ~~Tree 2: `circle_with_safety.xml` — Reactive safety fallback~~
- **Status:** Complete. ReactiveFallback with safety guard around mission sequence, safety land fallback.
- [x] Tree XML implemented
- [ ] Validate in SITL
- [ ] Test: trigger a geofence warning mid-circle, verify BT switches to safety land

#### ~~Tree 3: `multi_trajectory.xml` — Circle + figure-8 with retry~~
- **Status:** Complete. Uses `PreflightAndTakeoff` subtree, RetryNode(3) for each trajectory.
- [x] Tree XML implemented with reusable `PreflightAndTakeoff` subtree
- [ ] Validate in SITL — verify retry on trajectory failure

#### ~~Tree 4: `controller_switch_demo.xml` — Controller switch~~
- **Status:** Complete. Sequence: IsDependenciesReady → TakeoffAction → SetModeService(se3) → circle → SetModeService(passthrough) → LandAction.
- [x] Tree XML implemented
- [ ] Validate in SITL — verify controller switch mid-flight

### 9.4 ~~Subtree library~~
- **Status:** Complete. Two reusable subtrees in `peregrine_bt/trees/subtrees/`.
- [x] `PreflightAndTakeoff` — IsDependenciesReady → IsBatteryAbove → IsGpsHealthy → IsSafetyNominal → TakeoffAction with `{altitude_m}` blackboard port
- [x] `PreflightChecks` — same checks without takeoff (standalone preflight gate)
- [x] Stored in `peregrine_bt/trees/subtrees/`

### 9.5 ~~Launch integration~~
- **Status:** Complete. `bt_mission.launch.py` created in `peregrine_bringup`.
- [x] Create `bt_mission.launch.py`: includes `core_stack.launch.py` (optional via `start_core_stack` arg), launches `BTExecutorNode` as a `LifecycleNode` (separate process)
- [x] Launch arguments: `tree_file` (required), `tick_rate_hz` (default 2.0), `start_core_stack` (default true — set false when stack is already running on hardware)
- [x] `core_stack.launch.py` unchanged — all its args pass through from `bt_mission.launch.py`

---

## Phase 10: BT Testing & Operational Readiness

### 10.1 Temporal hierarchy compliance
The BT operates at the **slowest layer** (1-10Hz). It must NOT:
- Bypass the FSM by sending raw PX4 commands
- Hold references to high-frequency data (subscribe, snapshot, release)
- Block on long computations during tick (offload to async action nodes)
- Assume tick timing is deterministic (use timeouts, not tick counts)

- [ ] All action nodes verified async (return RUNNING, never block tick thread)
- [ ] All condition nodes verified non-blocking (read cached subscriber values via `behaviortree_ros2` registry, not per-tick subscriptions)
- [ ] Document the tick rate contract: "BT is safe to stutter or pause without affecting flight safety — the FSM and safety_monitor handle real-time concerns"
- [ ] Measure worst-case tick duration under load — must stay under 50ms at 2Hz tick rate

### 10.2 Unit tests
- [ ] Test each condition node in isolation: mock ROS topic data, verify SUCCESS/FAILURE thresholds
- [ ] Test each action node in isolation: mock ROS action/service server, verify RUNNING→SUCCESS and RUNNING→FAILURE paths
- [ ] Test condition nodes return FAILURE when no message has been received (null message path)

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

### 10.6 Documentation
- [ ] `peregrine_bt/README.md`: package overview, how to write a custom tree, how to add new BT nodes
- [x] Document the FSM vs BT separation of concerns — see `docs/ARCHITECTURE.md`
- [x] Document the temporal hierarchy — see `docs/ARCHITECTURE.md`
- [ ] Add example: "How to create a new mission" walkthrough (copy a tree XML, customize parameters, launch)

---

## Phase 11: TUI Command Interface (GCS → UAV)

Add command capability to the TUI so operators can send basic flight commands from the GCS terminal. Currently the GCS is read-only (Zenoh bridges only carry topics UAV → GCS). This phase opens the reverse path for actions and services.

### 11.1 Zenoh bridge allow-list updates
Open the existing Zenoh bridges to carry actions and services GCS → UAV. No new bridges or containers needed.

- [ ] `uav_bridge.json5`: add `action_servers` entries (`".*/uav_manager/takeoff"`, `".*/uav_manager/land"`)
- [ ] `uav_bridge.json5`: add `service_servers` entries (`".*/arm"`, `".*/set_mode"`, `".*/uav_manager/clear_emergency"`)
- [ ] `generate_gcs_config.py`: add matching `action_clients` and `service_clients` to the GCS bridge template
- [ ] Regenerate `gcs_bridge.generated.json5`
- [ ] Verify round-trip: GCS action client → Zenoh → UAV action server (test with `ros2 action send_goal` from GCS container)

### 11.2 TUI command clients
Add action and service clients to `tui_node`. The TUI already has the correct `uav_namespace` parameter — clients resolve under the same namespace.

- [ ] Add action clients: `uav_manager/takeoff`, `uav_manager/land`
- [ ] Add service clients: `arm`, `set_mode`, `uav_manager/clear_emergency`
- [ ] Non-blocking dispatch: send goal/request, track status, never block the TUI render loop
- [ ] Command timeout handling (Zenoh latency + server responsiveness)

### 11.3 TUI keybindings and display
- [ ] Keybindings: `T`=takeoff, `L`=land, `A`=arm/disarm, `M`=cycle mode, `E`=clear emergency
- [ ] Confirmation for destructive commands (double-press or shift key for arm/takeoff)
- [ ] Command status line in footer: show pending command, result (SUCCESS/FAILURE/TIMEOUT)
- [ ] Update footer help text with new keybindings

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
| Phase 7: BT package scaffolding & bridge | ~~1-2 weeks~~ **Done** | After Phases 0-2 |
| Phase 8: BT node implementation | ~~1-2 weeks~~ **Done** | After Phase 7 |
| Phase 9: Mission trees & executor | ~~1-2 weeks~~ **Done** | After Phase 8 |
| Phase 10: BT testing & operational readiness | 1-2 weeks | After Phase 9 |
| Phase 11: TUI command interface | 2-3 days | Optional / after Phase 10 |
