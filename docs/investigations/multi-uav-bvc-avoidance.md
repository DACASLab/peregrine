# Inter-UAV Collision Avoidance — Buffered Voronoi Cell (BVC) Pipeline

**Status:** PLAN — design + code-path sketches. Not yet implemented.
**Author:** design pass, 2026-06-10.
**Scope:** decentralized, per-UAV inter-agent collision avoidance for surveillance
(same-grid / adjacent-grid lawnmower sweeps). Modifies the position setpoint feeding the
PX4 passthrough controller. **Static-obstacle / perception avoidance is out of scope.**

> Decisions locked with the user (2026-06-10):
> 1. **Shared frame:** use the *existing* `map` frame (already implemented; see below).
> 2. **Injection point:** a **new `multi_agent_coordinator` node** that intercepts and
>    republishes `trajectory_setpoint` (no edits to trajectory/control managers).
> 3. **Geometry:** UAVs survey at the **same altitude** → BVC runs in **2D (x, y)**;
>    altitude held; an altitude-band gate disables avoidance between vertically separated
>    agents (transit/takeoff).
> 4. **Deadlock:** **right-hand-rule** tangential perturbation.

---

## 0. Ground truth from the code (not the docs)

The `docs/` describe a `multi_agent_coordinator` with a `CollisionConstraint` message —
**none of that exists**. What *does* exist and what this plan builds on:

### Control path (verified)
```
trajectory_manager (50 Hz)                 control_manager (250 Hz)        hardware_abstraction
  generators.sample() ──► /trajectory_setpoint ──► cache + passthrough ──► /control_output ──► PX4
  (ENU, odom frame)        (TrajectorySetpoint)     (re-run @250Hz)         (ENU→NED)        (closes pos loop)
```
- `trajectory_manager_node.cpp:427-429`: setpoint stamped with `header.frame_id = odomFrame_`.
- Coverage sweep uses `WaypointTrajectoryGenerator` → **position + velocity feed-forward +
  forward yaw** (`generators.hpp:230-261`). Hover/goto/takeoff use position+yaw.
- `control_manager_node.cpp:262-298`: holds the **last cached setpoint**; if none arrives it
  synthesizes a position hold. ⇒ if the coordinator stops publishing, the vehicle holds. Good
  failsafe.
- Default controller is `Px4PassthroughController` (`control_manager_parameters.yaml:34`) —
  **PX4 closes the position loop**. We only need to shape the position (+FF) setpoint.

### Frames (verified — this is the linchpin)
`frame_transformer.cpp`:
- `worldFrame_`/`mapFrame_` are **NOT prefixed** with the UAV namespace
  (`:94-95`); only `odom`/`base_link` are (`:96-98`). ⇒ `world` and `map` are **shared fleet
  frame names**.
- `world → map` is **identity** (`:144-149`).
- `map → odom` is a per-UAV **pure translation** = the UAV's GPS offset from the shared home
  datum (`tryInitHome`, `:287-303`): `geodeticToEnu(home_lat, home_lon, gnss.lat, gnss.lon)`.
  `home_lat/lon` are the same configured datum for every UAV (`13.018526, 77.565041`).
- ⇒ **`map` is a common fleet frame, anchored at the configured datum, and `map↔odom` is a
  translation with identity rotation.** Transforming a setpoint odom→map→odom is just adding /
  subtracting a constant vector — exact, no rotation, negligible cost.

> Caveat to validate in SITL: each PX4 EKF origin is set at the vehicle's own spawn point, so
> `map→odom` differs per UAV — which is exactly what makes `map` the right common frame. As
> long as every `frame_transformer` initializes home against the *same* `home_lat/lon` datum
> (it does), all `map` frames coincide. This is the single assumption the whole pipeline rests
> on; the SITL test in §11 asserts it against Gazebo ground truth.

### Transport (verified)
- Each UAV runs on its **own `ROS_DOMAIN_ID`** (`multi_uav_sitl.launch.py:171,187-189`,
  `ROS_LOCALHOST_ONLY=1`) → DDS-isolated. On hardware, separate Jetsons.
- A **Zenoh fleet bus** joins them (`docker/config/zenoh/uav_bridge.json5`). It already
  **exports** `/uav{ID}/estimated_state`, `/tf`, `/tf_static`, etc., but **imports nothing**
  (`subscribers: []`). To receive neighbor data we must add an import rule (§4).

---

## 1. Architecture

One `multi_agent_coordinator` node **per UAV**, composed into the existing
`peregrine_container`. It sits *between* trajectory_manager and control_manager on the
setpoint topic:

```
trajectory_manager ──► /uavN/trajectory_setpoint_raw ──► multi_agent_coordinator ──► /uavN/trajectory_setpoint ──► control_manager
                                                              ▲
                                  /uavN/estimated_state ──────┤  (own pose, odom frame; →map via static map→odom)
                                  tf: map→odom (static) ──────┤
              /uav*/multi_agent/fleet_state (via Zenoh) ──────┘  (neighbor poses, already in map frame)
                                                              │
                                  /uavN/multi_agent/fleet_state ──► (Zenoh export: own state to neighbors)
                                  /uavN/multi_agent/bvc_status  ──► (debug: active constraints, blocked flag)
```

The interception is done purely by **topic remap in the launch file** (§9) — trajectory_manager
publishes `trajectory_setpoint_raw`, control_manager still subscribes `trajectory_setpoint`.
Neither manager changes.

### Why a separate node (vs. inside trajectory/control manager)
- trajectory_manager is domain-isolated and has no neighbor knowledge; embedding fleet logic
  there couples generation with coordination.
- control_manager runs at 250 Hz and is plugin-based; fleet logic doesn't belong in the
  controller hot path.
- A standalone node matches the manager-per-domain philosophy, is independently testable, and
  fails safe (stop publishing → control_manager holds).

---

## 2. New interface message

`peregrine_interfaces/msg/FleetAgentState.msg` — the lightweight broadcast each coordinator
emits and consumes. **Everything is in the shared `map` frame** so no cross-agent TF is needed.

```
std_msgs/Header header          # stamp = sample time; frame_id = "map"
string  uav_id                  # e.g. "uav1"
geometry_msgs/Point  position   # map-frame ENU position (x,y,z)
geometry_msgs/Vector3 velocity  # map-frame ENU velocity (for prediction)
float64 yaw                     # ENU heading
uint8   mode                    # mirrors UAVState.state (IDLE/HOVERING/FLYING/...)
geometry_msgs/Point goal        # current setpoint target in map frame (intent; optional)
bool    avoidance_active        # is this agent currently doing BVC (for symmetry/debug)
```

> **Implementation note:** every UAV publishes/subscribes this on a SINGLE global topic
> `/fleet/agent_state` (absolute name, namespace-independent; `uav_id` is inside the message).
> This avoids ROS 2's inability to wildcard-subscribe and the need to discover per-namespace
> topic names — Zenoh merges all peers' publications on the shared topic, and each coordinator
> filters out its own `uav_id`. Debug/diagnostics are published as a `ManagerStatus` on
> `multi_agent/status` (no dedicated message type).

Rationale for broadcasting instead of consuming `/uav*/estimated_state` directly:
- `estimated_state` is in each neighbor's **odom** frame; converting it requires that
  neighbor's `map→odom` (extra TF plumbing over the bus). Broadcasting **map-frame** state
  pushes that one cheap translation to the owner and keeps the wire payload tiny.
- Carries **velocity + intent** for prediction and right-hand-rule symmetry.

(We keep the existing `/uav*/estimated_state` Zenoh export as-is for the GCS/monitoring;
the coordinator does not depend on importing it.)

---

## 3. New package layout

```
multi_agent_coordinator/
├── package.xml                 # depends: peregrine_interfaces rclcpp rclcpp_components
│                               #          tf2_ros tf2_eigen geometry_msgs Eigen3
├── CMakeLists.txt              # composable component, mirrors compute_monitor/CMakeLists.txt
├── include/multi_agent_coordinator/
│   ├── multi_agent_coordinator_node.hpp
│   ├── bvc_calculator.hpp      # pure geometry, NO ROS — unit-testable
│   └── neighbor_tracker.hpp    # neighbor store + staleness
├── src/
│   ├── multi_agent_coordinator_node.cpp
│   ├── bvc_calculator.cpp
│   ├── neighbor_tracker.cpp
│   └── multi_agent_coordinator_parameters.yaml   # generate_parameter_library
└── test/
    ├── test_bvc_calculator.cpp # half-plane build, projection, RH-rule, infeasible recovery
    └── test_neighbor_tracker.cpp
```

The geometry (`bvc_calculator`) is **ROS-free** so it can be unit-tested exactly like
`so3_utils` / `coverage_planner` already are.

---

## 4. Zenoh bridge change

Add the shared global topic to the bridge so each UAV exports its own state and imports
neighbors' on the same topic. In `docker/config/zenoh/uav_bridge.json5` (as implemented):

```json5
allow: {
  publishers:  [ /* ...existing... */ "/fleet/agent_state" ],
  subscribers: [ "/fleet/agent_state" ],
}
```

This is the only infra change. Payload is ~120 B at 20–50 Hz × N agents — trivial.

---

## 5. BVC geometry (`bvc_calculator`) — the math

All in 2D `map` frame; `r_s` = per-agent safety radius (half the desired min center-to-center
separation, including vehicle radius + margin). If both agents respect their own buffered cell,
separation ≥ `2·r_s`.

### Half-plane for self `p_i` against neighbor `p_j`
```
d   = p_j - p_i
dist= ||d||
n   = d / dist                       # unit normal, points toward the neighbor
b   = n·p_i + dist/2 - r_s           # offset; constraint is  n·x ≤ b
```
`p_i` satisfies `n·p_i ≤ b` iff `dist ≥ 2·r_s`. If `dist < 2·r_s` the agents are already too
close — the cell **excludes** the current position (infeasible). Recovery: drop the projection
and command a **retreat** straight along `-n` (away from the neighbor) until `dist ≥ 2·r_s`
(see §6 infeasible path).

### Predicted neighbor position (planning horizon `T`)
```
p_j_pred = p_j + v_j * T_eff         # T_eff inflated for stale neighbors (§7)
```
Use `p_j_pred` when building the half-plane so we react to closing velocity, not just current
gap.

### Projecting the desired setpoint onto the BVC
Given desired `p_d` (map frame) and the set of half-planes `{nₖ·x ≤ bₖ}`:
- If `p_d` satisfies all → return `p_d` (no-op; the common case).
- Else compute the Euclidean projection onto the convex intersection. For the small constraint
  counts here (typically 1–4 active neighbors) use **Dykstra's alternating projection** — it
  converges to the *true* projection onto the intersection (naive "project onto most-violated,
  repeat" does not handle corners correctly):

```cpp
// bvc_calculator.cpp  (sketch)
Eigen::Vector2d projectOntoBVC(Eigen::Vector2d p_d,
                               const std::vector<HalfPlane>& planes,
                               int max_iter = 20, double tol = 1e-4) {
  if (satisfiesAll(p_d, planes)) return p_d;
  Eigen::Vector2d x = p_d;
  std::vector<Eigen::Vector2d> q(planes.size(), Eigen::Vector2d::Zero()); // Dykstra corrections
  for (int it = 0; it < max_iter; ++it) {
    Eigen::Vector2d x_prev = x;
    for (size_t k = 0; k < planes.size(); ++k) {
      Eigen::Vector2d y = x + q[k];
      Eigen::Vector2d xk = projectHalfPlane(y, planes[k]);  // y - max(0, n·y - b) n
      q[k] = y - xk;
      x = xk;
    }
    if ((x - x_prev).norm() < tol) break;
  }
  return x;
}
// projectHalfPlane: s = n·y - b; return s <= 0 ? y : y - s * n;
```

### Velocity feed-forward clamp (critical for coverage sweep)
The sweep setpoint carries velocity FF. After projection, for every **active** constraint
(`nₖ·x_safe ≈ bₖ`), remove the FF component pushing *into* the boundary so the FF doesn't fight
the projection:
```cpp
for (k : active) if (n_k.dot(v) > 0) v -= (n_k.dot(v)) * n_k;   // project v onto the tangent
```

---

## 6. Node behavior (`multi_agent_coordinator_node`)

Event-driven on the raw setpoint (50 Hz), projecting against the freshest neighbor snapshot.

```cpp
// onRawSetpoint(TrajectorySetpoint raw)   — sketch
if (!own_state_ || !mapToOdom_) { passthroughHold(raw); return; } // not ready → hold

// 1. lift to map frame (translation only: map = odom + mapToOdomOffset)
TrajectorySetpoint out = raw;                       // preserve z, yaw, flags, stamp
Eigen::Vector2d p_i = own_map_xy_;                  // from estimated_state(odom)+offset
double z_i = own_map_z_;
Eigen::Vector2d p_d(raw.position.x + off_.x, raw.position.y + off_.y);   // desired, map xy

// 2. assemble half-planes from neighbors within range AND altitude band
std::vector<HalfPlane> planes;
for (auto& nb : tracker_.neighbors(now)) {
  if (std::abs(nb.z - z_i) > altitude_band_) continue;        // vertically deconflicted → skip
  Eigen::Vector2d pj = nb.xy + nb.vxy * T_eff(nb);            // predicted
  if ((pj - p_i).norm() > sensing_range_) continue;
  planes.push_back(makeHalfPlane(p_i, pj, r_s(nb)));          // r_s inflated if stale (§7)
}

// 3. infeasible (already too close to someone) → retreat, skip normal projection
if (auto esc = firstViolatedAtOwnPos(p_i, planes)) {
  p_d = p_i - esc->n * retreat_step_;          // straight away from the offender
}

// 4. project, with right-hand-rule if blocked (§ below)
Eigen::Vector2d p_safe = projectOntoBVC(p_d, planes);
p_safe = applyRightHandRule(p_i, p_d, p_safe, planes);   // no-op unless blocked

// 5. velocity FF clamp on active constraints
clampVelocityFF(out.velocity, p_safe, planes);

// 6. lower back to odom (subtract offset) and republish
out.position.x = p_safe.x() - off_.x;
out.position.y = p_safe.y() - off_.y;
// z, yaw untouched (same-altitude 2D); header.frame_id stays odom
pub_->publish(out);
publishBvcStatus(planes, blocked);
```

`passthroughHold` republishes a zero-velocity position-hold at the current pose if we can't
compute (missing own state / offset) so control_manager never starves and never gets an
unprojected setpoint.

### Right-hand-rule (deadlock / livelock)
```cpp
// applyRightHandRule — sketch
Eigen::Vector2d d = (p_d - p_i);
double want = d.norm();
double made = (p_safe - p_i).dot(d.normalized());     // progress toward goal
bool neighbor_ahead = anyNeighborInCone(p_i, d, planes, cos_half_angle_);
if (want > eps_ && made < progress_frac_ * want && neighbor_ahead) {
  Eigen::Vector2d dir = d.normalized();
  Eigen::Vector2d right(dir.y(), -dir.x());           // ENU: right of travel
  Eigen::Vector2d p_bias = p_safe + rh_gain_ * right; // nudge along boundary, to the right
  return projectOntoBVC(p_bias, planes);              // re-project so it stays feasible
}
return p_safe;
```
Both agents in a head-on conflict bias to their right consistently → they slide past each other.
`rh_gain_` decays to zero once `made ≈ want` (clear), so it doesn't perturb free flight.

---

## 7. Neighbor tracking & failure handling (`neighbor_tracker`)

Each neighbor entry: last `FleetAgentState`, receive time, derived staleness.
- `age ≤ stale_timeout`  → **fresh**: nominal `r_s`, `T_eff = horizon`.
- `stale_timeout < age ≤ lost_timeout` → **stale**: inflate `r_s *= stale_mult`, extend
  `T_eff` (dead-reckon on last velocity), keep avoiding.
- `age > lost_timeout` → **lost**: keep the last position as a static obstacle with inflated
  `r_s`, **and** raise a diagnostic (optionally a `safety_monitor` heartbeat-style alert). We
  do *not* silently drop a lost neighbor — dropping it would re-open the conflict region.
- Own id is always filtered out.

This conservative stale/lost behavior is applied regardless of the deadlock policy; it is
orthogonal to the right-hand-rule.

---

## 8. Altitude band gate

Per the same-altitude decision, BVC normally engages for all surveilling agents. The
`altitude_band` gate (`|z_i − z_j| > band → skip`) makes the pipeline robust during phases
where altitudes legitimately differ (staggered takeoff, transit to/from sectors, one agent
RTL-climbing) so we don't fight phantom horizontal conflicts between vertically separated
agents. Default band a little larger than expected altitude-hold error (e.g. 1.5–2 m).

---

## 9. Launch wiring (no manager code changes)

In `core_stack.launch.py`, add the coordinator to `peregrine_container` and remap
trajectory_manager's output:

```python
ComposableNode(
    package="trajectory_manager", plugin="trajectory_manager::TrajectoryManagerNode",
    name="trajectory_manager", namespace=uav_namespace,
    remappings=[("trajectory_setpoint", "trajectory_setpoint_raw")],   # NEW
    parameters=[config_overrides, {"use_sim_time": use_sim_time}],
),
ComposableNode(   # NEW
    package="multi_agent_coordinator",
    plugin="multi_agent_coordinator::MultiAgentCoordinatorNode",
    name="multi_agent_coordinator", namespace=uav_namespace,
    parameters=[config_overrides, {
        "uav_id": uav_namespace,            # e.g. "uav1"
        "use_sim_time": use_sim_time,
        "enabled": True,                    # master switch (False → pure passthrough)
    }],
),
```
- `enabled: False` → node is a transparent passthrough (`raw → trajectory_setpoint`), so the
  single-UAV path and A/B testing are trivial.
- control_manager is untouched (still subscribes `trajectory_setpoint`).

---

## 10. Parameters (`multi_agent_coordinator_parameters.yaml`)

```yaml
multi_agent_coordinator:
  uav_id:            {type: string, default_value: "uav1"}
  enabled:           {type: bool,   default_value: true}   # false → transparent passthrough
  publish_rate_hz:   {type: double, default_value: 50.0}   # = trajectory rate; event-driven anyway
  broadcast_rate_hz: {type: double, default_value: 30.0}   # own fleet_state out
  # safety geometry
  safety_radius_m:   {type: double, default_value: 2.5}    # r_s; min separation = 2*r_s = 5 m
  sensing_range_m:   {type: double, default_value: 25.0}
  planning_horizon_s:{type: double, default_value: 1.5}
  altitude_band_m:   {type: double, default_value: 2.0}
  # neighbor staleness
  stale_timeout_s:   {type: double, default_value: 0.4}
  lost_timeout_s:    {type: double, default_value: 1.5}
  stale_buffer_mult: {type: double, default_value: 1.5}
  # right-hand-rule
  rh_progress_frac:  {type: double, default_value: 0.3}    # blocked if progress < frac*desired
  rh_cone_deg:       {type: double, default_value: 60.0}
  rh_gain_m:         {type: double, default_value: 1.0}
  # infeasible recovery
  retreat_step_m:    {type: double, default_value: 0.5}
  frame_map:         {type: string, default_value: "map"}
  frame_odom_suffix: {type: string, default_value: "odom"}
```
`safety_radius_m` aligns with the current `test_separation` target (≥5 m XY). The existing
2 m default in the (unused) docs is too tight for the airframe; 2.5 m → 5 m separation.

---

## 11. Testing

### Unit (`bvc_calculator`, ROS-free — matches existing test style)
- Half-plane: own pos feasible iff `dist ≥ 2·r_s`; correct `n`, `b`.
- Projection: single plane (point→boundary); two planes (corner) = exact Dykstra result;
  no-op when already feasible; idempotent.
- Velocity FF clamp removes only the into-boundary component on active constraints.
- Right-hand-rule: head-on symmetric pair both bias right → net lateral separation increases,
  no oscillation across ticks (simulate the discrete loop).
- Infeasible recovery: `dist < 2·r_s` → command points away from offender, `dist` grows.

### SITL (`multi_uav_sitl.launch.py`, 2 then 4 UAVs)
1. **Frame assertion (gating test):** with both UAVs hovering, assert each one's
   `multi_agent/fleet_state.position` (map frame) matches Gazebo ground-truth world position
   within GPS tolerance, and that the two `map` frames coincide. *If this fails, the shared-frame
   assumption in §0 is wrong and BVC must not be trusted* — fix frames first.
2. **Same-cell sweep:** two UAVs assigned overlapping/same grid cell at the same altitude
   (reuse `test_separation` sequences as the GoTo/coverage goals). Log min center-to-center
   distance; **assert it never drops below `2·r_s − ε`**. Compare `enabled:false` (baseline,
   expect violations) vs `enabled:true` (expect respected).
3. **Head-on deadlock:** two UAVs swap positions across a shared cell at the same altitude.
   Assert both reach goals (no permanent stall) and separation respected — exercises the
   right-hand-rule.
4. **Comms loss:** kill one UAV's Zenoh bridge mid-sweep; assert the other inflates buffer
   (stale) then treats it as a static obstacle (lost) and still respects separation around the
   last-known position.

Add `tools/`-style script analogous to the existing SITL regression harness; emit a CSV of
pairwise distances for the assertions.

---

## 12. Phasing

1. **Interfaces + bridge:** `FleetAgentState.msg` (+ CMake), Zenoh import rule. Verify a UAV
   sees neighbor `fleet_state` over the bus (frame-assertion test #1).
2. **`bvc_calculator` + unit tests.** Pure geometry, fully tested before any ROS wiring.
3. **Node skeleton:** own-state lift to map frame, broadcast, transparent passthrough
   (`enabled:false`). Confirm zero behavior change single-UAV.
4. **Projection live (`enabled:true`):** position-only BVC; SITL same-cell test #2.
5. **Velocity FF clamp + right-hand-rule:** SITL deadlock test #3.
6. **Stale/lost handling + safety alert:** SITL comms-loss test #4.
7. **Tuning** `safety_radius/horizon/rh_gain` on hardware (conservative buffers first).

---

## 13. Known limitations / future

- **Accuracy is GPS-grade.** Relative position over `map` is only as good as each EKF's global
  fix (~GPS error; cm with RTK). Tight formation (<~2 m) needs UWB/vision on top — out of
  scope. `safety_radius` must cover GPS relative error budget.
- **2D only.** Vertical conflicts during transit are handled by the altitude-band gate, not by
  3D BVC. If mixed-altitude dynamic conflicts become real, extend half-planes to 3D and project
  in R³ — the structure is unchanged.
- **Best-effort comms.** Designed around stale/lost handling; never assumes delivery.
- **Not a static-obstacle planner.** Perception/obstacle avoidance is a separate layer; a
  detected static obstacle could later be injected as a zero-velocity virtual neighbor through
  the same half-plane machinery.
- Interaction with `safety_monitor` geofence/envelope: BVC only ever *shrinks* the reachable
  set; it cannot push a UAV outside the geofence (projection moves toward own side). Worth an
  explicit test once geofence + BVC run together.
```

