# PX4 Frames in the Peregrine Stack — Root Cause + Frame Redesign

> **DEPRECATED — DO NOT USE.** Superseded by
> [`world-frame-anchoring-plan.md`](world-frame-anchoring-plan.md), which is the current,
> self-contained source of truth (cleaner frame model: command in `world`, dynamic
> `world↔PX4-local` transform, generators unchanged). This file is kept only for history.

**Status:** SPEC / plan. Root cause is confirmed from PX4 source **and** flight-log data
(see §2 and Appendix A). The frame architecture in §4 supersedes the earlier
"re-anchor odom inside hardware_abstraction" framing — corrections now live in the TF
tree (`map→odom`, `world→map`), driven by PX4 `vehicle_local_position` data, and PX4's
own estimate is left untouched.

---

## 1. The symptom

On the ground, before takeoff, the height shown in the TUI (the per-UAV position
estimate) reads an arbitrary value and steps discontinuously — observed swinging on the
order of −10 m → +2 m while the vehicle is stationary. This corrupts takeoff and any
logic that assumes the estimate's `z = 0` is the ground, and blocks reliable autonomous
flight. The airframe has GPS.

A separate, cosmetic observation: QGC's `local_home_position` shows small repeated updates
in the mavlink console. This is **not** the same thing as the estimate jump (see §3.3).

---

## 2. Root cause (confirmed)

### 2.1 The ROS estimate Z is referenced to PX4's *mutable* vertical origin, not the ground

Both `vehicle_odometry.z` (what the stack consumes) and `vehicle_local_position.z` are the
**same** quantity — `EKF2` fills both from `_ekf.getPosition()`
(`EKF2.cpp:1700-1702` for lpos, `:1823` for odometry):

```
estimator_interface.cpp:628
const float z = -(lla.altitude() - getEkfGlobalOriginAltitude());
```

`getEkfGlobalOriginAltitude()` returns the EKF's vertical origin, or **0 if it is not yet
set**:

```
estimator_interface.h:328
return PX4_ISFINITE(_local_origin_alt) ? _local_origin_alt : 0.f;
```

`_local_origin_alt` is established from the **baro pressure altitude** at height-fusion
init (`baro_height_control.cpp:170` → `initialiseAltitudeTo`, `ekf_helper.cpp:219-241`),
and later re-set/shifted by other height-reference events. So the frame's `z = 0` is an
**arbitrary baro datum**, never tied to the ground, and it can move at runtime.

⇒ The reported estimate Z on the ground is whatever offset the EKF origin happens to have
this boot, and it steps whenever that origin is (re-)established.

### 2.2 The vertical origin is re-anchored repeatedly (data)

`z_reset_counter`/`delta_z` are written only from two functions — `resetAltitudeTo()` and
`updateVerticalPositionResetStatus()` — and **every** caller is inside the EKF:

- Origin altitude set/move: `ekf_helper.cpp:140, 145, 238`
- Height-source (re)start / all-sources-failing reset: `baro_height_control.cpp:137`,
  `gnss_height_control.cpp:114, 157`, `range_height_control.cpp`, `ev_height_control.cpp`,
  `fake_height_control.cpp`

Flight-log evidence (Appendix A): across nominally identical hovers the armed-window Z
range differs wildly per flight (one flight sits at `z ≈ −15 m` throughout, another at
`+11.7 m`), and `z_reset_counter` climbs within a boot session (…6, 8, 12, and 45 in one
log). The origin is genuinely arbitrary and non-repeatable.

### 2.3 The takeoff bug this produces

`TakeoffGenerator` treats its target altitude as an **absolute** Z in the estimator frame
and climbs `ground_z → target`:

```
generators.cpp (TakeoffGenerator)
startAltitude_(startState.pose.pose.position.z),   // ground z, in the EKF-origin frame
targetAltitude_(targetAltitudeM),                  // absolute, e.g. 2.0
...
sample.setpoint.position.z = startAltitude_ + direction*step;   // startAltitude_ → targetAltitude_
```

The target is passed straight through as absolute (`trajectory_manager_node.cpp:627`,
`goal.params[0]`). This silently assumes **`odom z = 0` is the ground**, which §2.1–2.2
prove is false. With ground at `−15`, "climb to 2" commands a **+17 m** climb; with ground
at `+11`, it commands a **9 m descent into the ground**. The whole stack inherits this
assumption for any absolute-Z command.

---

## 3. What was ruled out (so we don't revisit it)

### 3.1 In-flight EKF resets are not the driver
In the armed window of every available log, `z_reset_counter` is **constant** and
`vehicle_local_position.z` has **no** steps > 1.5 m. The disruptive re-anchoring happens
on the **ground/at boot** (logging starts at arm, so it is not even captured — see
Appendix A). Earlier ".ulog gate" analysis that concluded "no resets" only ever looked at
the armed window; it was correct about flight and wrong to generalize to the ground.

### 3.2 The ROS path does not manufacture the jump
`hardware_abstraction` converts `VehicleOdometry` NED→ENU faithfully
(`px4_hardware_abstraction.cpp:266`); `estimation_manager` is a pure passthrough
(`px4_passthrough_estimator`); `estimated_state.z` is shown directly by the TUI
(`tui_node.cpp:613`). `px4_msgs` and the firmware `VehicleOdometry.msg` are byte-compatible
(both `MESSAGE_VERSION = 0`). A smooth source therefore yields a smooth TUI — the jump
originates in PX4's origin handling (§2), not in ROS.

### 3.3 The `home_position` flicker is unrelated to the estimate
QGC's `local_home_position` updates come from the commander correcting the **home/RTL
altitude** from GPS/baro disagreement (`HomePosition.cpp:369-404`, `:395-396`
`home.alt -= offset; home.z += offset`). This calls **no** EKF reset, touches **neither**
`vehicle_local_position.z` nor `vehicle_odometry.z`, and bumps **no** reset counter. It is
cosmetic to the home datum and must **not** be used to anchor any ROS frame —
`home_position` is mutable by design.

---

## 4. Frame architecture (the redesign)

### 4.1 Principle

Use the frames PX4 already maintains; **do not** mutate PX4's estimate and **do not**
re-derive anchors from raw GNSS or `home_position`. PX4's local estimator frame is
internally consistent and is the frame its own controller closes the loop in — keep
`odom→base_link` as that raw estimate. Put all global anchoring and reset compensation in
the **upper TF edges**, driven by `vehicle_local_position` fields. The single thing PX4
does *not* provide — a ground-referenced `z = 0` — is handled as an explicit, separate
datum (§4.4), not smuggled into the EKF frame.

### 4.2 Frame tree

```
world ──(1)──► map ──(2)──► odom ──(3)──► base_link ──(4)──► base_link_frd
```

| # | Edge | Source (all from `vehicle_local_position` unless noted) | When it updates |
|---|------|----------------------------------------------------------|-----------------|
| 1 | `world→map` | `geodeticToEnu(world_datum, ref_lat, ref_lon, ref_alt)`, **gated on `xy_global && z_global`** | on `ref_timestamp` change (rare) |
| 2 | `map→odom` | accumulated EKF reset offset: `Σ delta_xy`, `Σ delta_z`, `Σ delta_heading` | **steps only on a `*_reset_counter` change**; identity otherwise |
| 3 | `odom→base_link` | raw PX4 local estimate (NED→ENU). **Untouched.** | every sample (smooth) |
| 4 | `base_link→base_link_frd` | static FLU↔FRD | static |

Field→edge split (important): **`ref_*` anchors `world→map`** (global placement of the EKF
origin); **the reset deltas drive `map→odom`** (keeping `odom` locally smooth). They are
*different* edges fed by *different* fields of the same message. The EKF origin *is* the
odom origin, so if `ref_*` were used for `map→odom` it would collapse to identity and the
reset handling would be lost.

### 4.3 Frame definitions

- **`odom`** — per-UAV PX4 local estimator frame (ENU). Locally smooth: continuous between
  EKF resets, with the only discontinuities being the discrete reset instants, which edge
  (2) cancels. Never compare two UAVs' `odom` frames directly.
- **`base_link`** — vehicle body (FLU), from PX4 odometry/attitude.
- **`map`** — globally-consistent, non-drifting per-UAV frame. Origin coincides with the
  EKF local origin; placed in `world` via `ref_*`. Valid only when `xy_global && z_global`.
  Vertically this is an **AMSL-based** frame (`ref_alt` is MSL) — correct for cross-UAV Z
  comparison, but **`z = 0` is the EKF origin, not the ground**.
- **`world`** — shared fleet/site frame. Defined only when a surveyed / global / external
  shared reference is valid (configured `world_datum_*`, or an external global-localization
  source). Single-agent fallback: `world == map` (use first valid `ref_*` as the datum).

### 4.4 The vertical ground datum (separate concern — this is what fixes takeoff)

Nothing in §4.2 makes "climb to 2 m" mean *2 m above ground*, because `ref_alt` is MSL and
the EKF origin is a baro datum. "Height above ground/takeoff" is its **own** datum:

- **Source (decision, see §7):** capture the estimate Z at arm / first-valid as the
  takeoff-ground reference, **or** use a downward range finder's `dist_bottom` if equipped.
  **Not** `home.z` (mutable).
- **Use:** trajectory/takeoff targets are expressed as **height above this ground datum**
  (relative), never as absolute estimator-frame Z. Equivalently, the ground datum can be
  published as a frame (e.g. `map→takeoff` / a ground offset) and altitude commands taken
  in it.

This keeps the two concerns cleanly separated: edges (1)(2) fix multi-agent + global
consistency; the ground datum fixes height semantics + takeoff. Conflating them is what
produced the bug.

---

## 5. Implementation plan (per package)

### 5.1 `hardware_abstraction` — switch state source, expose the anchor
Files: `src/peregrine_core/hardware_abstraction/src/px4_hardware_abstraction.cpp`,
`.../include/hardware_abstraction/px4_hardware_abstraction.hpp`

- Replace the `VehicleOdometry` subscription with `VehicleLocalPosition`
  (`/fmu/out/vehicle_local_position` + `getMessageNameVersion<>()`), plus `VehicleAttitude`
  and `VehicleAngularVelocity` for orientation/rates. Publish the ROS `Odometry`/`State`
  on the lpos callback using the latest attitude/rates snapshots.
  - velocity: lpos `v*` is **NED world** → `v_body_frd = q_ned_frd^{-1} * v_ned`;
    `v_body_flu = frdToFlu(v_body_frd)`.
  - covariance from `eph/epv` (`evh/evv` for twist).
- The published `odom→base_link` estimate stays the **raw** PX4 local position (edge 3 is
  untouched — no smoothing, no offset applied here).
- Maintain a reset accumulator, **gated on counter change (not on `delta != 0`)** —
  `delta_*` is last-reset-only, not cumulative:
  - on `xy_reset_counter` change: `offset.xy += delta_xy`
  - on `z_reset_counter` change: `offset.z += delta_z`
  - on `heading_reset_counter` change: `offset.heading = wrap_pi(offset.heading + delta_heading)`
  - re-baseline if `ref_timestamp` changes or counters jump backward (EKF reinit).
- Capture the ground datum (arm / first-valid Z, or `dist_bottom`).
- Publish a new `FrameAnchor` message (reliable, low-rate / on-change) carrying the offset,
  `ref_lat/lon/alt`, `ref_valid = xy_global && z_global`, `ref_timestamp`, and the ground
  datum.

### 5.2 `peregrine_interfaces` — new message
Files: `src/peregrine_core/peregrine_interfaces/msg/FrameAnchor.msg` (+ `CMakeLists.txt`)

```
std_msgs/Header header
geometry_msgs/Vector3 map_to_odom_enu   # = nedToEnu(offset.xy, offset.z); steps on reset
float64 map_to_odom_yaw                  # accumulated heading offset (ENU)
float64 ref_lat
float64 ref_lon
float64 ref_alt                          # AMSL
bool    ref_valid                        # xy_global && z_global
uint64  ref_timestamp
float64 ground_datum_z                   # estimator-frame Z taken as ground (or via dist_bottom)
bool    ground_datum_valid
```

### 5.3 `frame_transforms` — TF tree from the anchor
Files: `src/peregrine_core/frame_transforms/src/frame_transformer.cpp`,
`.../include/frame_transforms/frame_transformer.hpp`,
`.../src/frame_transforms_parameters.yaml`

- **Remove** the GNSS/`home_lat/lon` init entirely (`frame_transformer.cpp:240-310`, the
  `onGnss`/`onGpsStatus`/`tryInitHome` path and the `alt = 0` `geodeticToEnu` at `:287`).
- Subscribe to `FrameAnchor`. Publish:
  - `world→map` = `geodeticToEnu(world_datum, ref_*)` when `ref_valid` (refresh on
    `ref_timestamp`); identity (`world==map`) until a datum/ref is available.
  - `map→odom` = `map_to_odom_enu` / `map_to_odom_yaw` (dynamic, steps on reset).
  - `odom→base_link` from the estimate (unchanged); `base_link→base_link_frd` static.
- Params: drop `home_*` and `gps.*`; add `world_datum_lat/lon/alt`.
- Replace the equirectangular `geodeticToEnu` with PX4's azimuthal-equidistant projection,
  replicated inline in `conversions.hpp` so `world↔ref` math matches the EKF's
  `MapProjection`; unit-test both against PX4 reference values.

### 5.4 `trajectory_manager` — fix height semantics (the takeoff fix)
Files: `src/peregrine_core/trajectory_manager/src/generators.cpp`,
`.../src/trajectory_manager_node.cpp`

- Stop commanding absolute estimator-frame Z. Express takeoff (and other altitude targets)
  as **height above the ground datum**:
  - minimal: `TakeoffGenerator` target becomes a height *delta* —
    `target_z = startAltitude_ + height` (climb `height` above current ground), instead of
    an absolute `targetAltitude_`.
  - principled: take altitude targets in the ground-anchored frame and transform down to
    `odom` before the setpoint goes to PX4.
- Setpoint re-anchoring across resets: when commanding through `map→odom`, the outgoing
  `odom` setpoint shifts by the reset delta automatically (the transform moved). Keep PX4's
  own reset net **on**; the only residual is one bridge-latency window, self-correcting and
  never cumulative. Keep `nowMicros()` timestamps; do **not** disable timesync.

### 5.5 Untouched
`estimation_manager`, `control_manager`, `tui_status` keep consuming `estimated_state`. No
changes needed — this is the payoff of keeping `odom` as the raw PX4 frame and confining
corrections to the TF tree + the anchor message.

---

## 6. Tests

- **Unit (`hardware_abstraction`):** synthetic lpos sequences (single/multiple/simultaneous
  xy+z, heading, counter wrap, `ref_timestamp` change) ⇒ raw `odom` continuous between
  resets; `FrameAnchor` offset = `Σδ`; ground datum captured once and stable.
- **Unit (`frame_transforms`):** `FrameAnchor` → expected `world→map` (AMSL Z; ref change;
  datum-from-first-ref) and `map→odom` (steps on reset). `geodeticToEnuAzEq` vs PX4
  reference values.
- **SITL:** offboard hold + induced resets (Z: `EKF2_HGT_REF` toggle; XY: `EKF2_GPS_CTRL`)
  ⇒ counter ticks, `map→base_link` continuous, Gazebo ground-truth unmoved. Takeoff from a
  field whose EKF origin is offset from ground ⇒ vehicle climbs the commanded height above
  ground (not the origin).
- **SITL multi-UAV:** two agents at known offset; B-relative-to-A in `world` matches
  ground-truth within GPS tol; Z agreement via AMSL.
- **`view_frames`:** `world→map→odom→base_link`; `map→odom` steps on induced resets.

---

## 7. Open decisions

1. **Ground datum source:** capture-at-arm (simple, no extra HW) vs. range finder
   `dist_bottom` (true AGL, needs a downward sensor) vs. configured. Default: capture-at-arm,
   upgrade to `dist_bottom` where equipped.
2. **Where altitude targets live:** relative-height quick fix in `TakeoffGenerator` (unblocks
   flight immediately) vs. full "command in ground-anchored frame, transform to `odom`"
   (cleaner, larger change). Recommend shipping the relative fix first, then the frame route.
3. **`world_datum`:** configured site origin (`world_datum_lat/lon/alt`, shared per fleet),
   falling back to first valid `ref_*` for single-agent.

## 8. Sequencing

1. **Takeoff height fix** (`trajectory_manager`, §5.4 minimal) — unblocks flight today;
   independent of the frame work.
2. **State source + `FrameAnchor`** (`hardware_abstraction`, §5.1–5.2) — raw `odom` +
   anchor; no downstream changes.
3. **TF tree** (`frame_transforms`, §5.3) — `world→map`, dynamic `map→odom`; enables
   multi-agent + correct RViz/world.
4. **Command-in-anchored-frame** (`trajectory_manager`, §5.4 principled) — optional polish
   once 1–3 are validated.

---

## Appendix A — flight-log evidence

Tooling: `pyulog` over `px4_logs_peregrine/` and `peregrine_px4_logs_2/`.

- **Every log starts at arm** (`pre-ground span ≈ −0.0 .. −0.3 s`). The boot/ground origin
  establishment is never captured, which is why armed-window analysis sees a constant
  counter and missed the cause.
- **Per-flight Z origin is arbitrary** (armed window, nominally identical hovers):
  `log_41 ≈ −15 m` throughout, `log_30 ≈ +11.7 m`, `log_31 −5..+12`, `log_46 −3..+9.7`.
- **`z_reset_counter` climbs within a boot session:** 6, 8, 12, and 45 (`log_38`).
- **Armed window is clean:** counter constant, no `vehicle_local_position.z` step > 1.5 m —
  confirming the disruption is ground/boot origin (re)establishment, not in-flight resets.
- The large Z excursions seen *during AUTO_LAND* (e.g. `+6.8 m` in `log_7`) are the
  land-detector / stale-setpoint issue (see `stale-setpoint-after-managed-land.md`), a
  separate problem from the frame anchoring covered here.

## Appendix B — key source references (PX4 + Peregrine)

PX4 (`/scratch/robotics/quad-firmware/PX4-Autopilot`):
- `getPosition()` Z = `-(alt - origin_alt)`: `src/modules/ekf2/EKF/estimator_interface.cpp:628`
- origin alt or 0: `.../estimator_interface.h:328`
- origin alt set from baro: `.../aid_sources/barometer/baro_height_control.cpp:170`;
  `.../ekf_helper.cpp:219-241`
- z-reset writers (all in EKF): `.../position_fusion.cpp:185,222`; callers `ekf_helper.cpp:140,145,238`,
  `baro_height_control.cpp:137`, `gnss_height_control.cpp:114,157`, `range_height_control.cpp`,
  `ev_height_control.cpp`, `fake_height_control.cpp`
- lpos `ref_*` / `xy_global` / `z_global` / `delta_*` / counters: `src/modules/ekf2/EKF2.cpp:1728-1781`
- odometry == lpos position: `EKF2.cpp:1700-1702`, `:1823`
- home alt correction (commander-side, off the odom path): `src/modules/commander/HomePosition.cpp:369-404`

Peregrine (`/scratch/robotics/peregrine`):
- consumes `VehicleOdometry`, faithful NED→ENU: `.../hardware_abstraction/src/px4_hardware_abstraction.cpp:175,266`
- estimation passthrough: `.../estimation_manager/src/px4_passthrough_estimator.cpp`
- TUI shows `estimated_state.z`: `.../tui_status/src/tui_node.cpp:613`
- GNSS/home frame hack (to remove): `.../frame_transforms/src/frame_transformer.cpp:240-310` (`alt=0` at `:287`)
- takeoff absolute-Z assumption: `.../trajectory_manager/src/generators.cpp` (TakeoffGenerator),
  `.../trajectory_manager/src/trajectory_manager_node.cpp:627`
