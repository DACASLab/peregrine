# World-Frame Anchoring — Frame Architecture & Implementation Plan

**Audience:** an engineer/agent starting this work cold. This doc is self-contained.
**Status:** SPEC / plan, ready to implement. Root cause is confirmed from PX4 source *and*
flight-log data (Appendix A/B). This supersedes
`px4-localposition-reset-frames.md` (now deprecated).

**One-line thesis:** PX4's local estimator frame has a runtime-mutable origin; our stack
wrongly treats it as a fixed Cartesian frame and caches local setpoints in it. The fix is to
define a **fixed, physically-anchored world frame**, keep the `world↔PX4-local` transform
**dynamic** (driven by PX4 data), command goals **in world**, and project to PX4's local
frame **every cycle** — which is exactly what PX4's own navigator does internally.

---

## 1. Symptom

On the ground before takeoff, the per-UAV position estimate (shown in the TUI) reads an
arbitrary value and steps discontinuously — observed swinging ~ −10 m → +2 m in Z while the
vehicle is stationary. This corrupts takeoff and any logic that assumes the estimate's
`z = 0` is the ground, and blocks reliable autonomous flight. The airframe has GPS.

(Cosmetic, unrelated: QGC's `local_home_position` shows small repeated updates — that's the
commander correcting the home/RTL altitude, see §3.)

---

## 2. Root cause (confirmed)

### 2.1 The estimate's Z is referenced to PX4's *mutable* vertical origin, not the ground
Both `vehicle_odometry.z` (what we consume) and `vehicle_local_position.z` are the same
quantity, `EKF2` filling both from `_ekf.getPosition()`:

```
estimator_interface.cpp:628   z = -(lla.altitude() - getEkfGlobalOriginAltitude());
estimator_interface.h:328     getEkfGlobalOriginAltitude() = isfinite(_local_origin_alt) ? _local_origin_alt : 0;
```

`_local_origin_alt` is set from the **baro pressure altitude** at height-fusion init
(`baro_height_control.cpp:170`, `ekf_helper.cpp:219-241`) and re-set/shifted by later
height-reference events. So the local frame's `z = 0` is an **arbitrary baro datum**, never
the ground, and it moves at runtime.

### 2.2 The origin is re-anchored repeatedly (data)
`z_reset_counter`/`delta_z` are written only by `resetAltitudeTo()` /
`updateVerticalPositionResetStatus()`, and **every** caller is inside the EKF (origin
set/move; height-source (re)start or all-sources-failing reset). Flight logs (Appendix A):
per-flight Z origin is arbitrary (one hover sits at `z ≈ −15 m`, another at `+11.7 m`), and
`z_reset_counter` climbs within a boot session (…6, 8, 12, 45). The origin is genuinely
non-repeatable.

### 2.3 We cache a local setpoint in that moving frame
`TakeoffGenerator` captures a **local** `startPosition_.z` once and commands an **absolute**
local target (`generators.cpp`; target passed straight through at
`trajectory_manager_node.cpp:627`). This assumes `odom z = 0` is the ground. With ground at
−15, "climb to 2" = +17 m climb; with ground at +11, = 9 m **descent into the ground**.
Every absolute-Z command in the stack inherits the bad assumption.

---

## 3. What was ruled out (don't revisit)
> **NOTE — in-flight resets are NOT ruled out.** Earlier drafts claimed "no in-flight EKF
> resets." That was wrong (an overgeneralization from `log_6/7`). The logs show
> `xy/z/heading_reset_counter` incrementing **during armed flight** — `log_42` @+3 s after
> arm, `log_44` deep in flight (+131 s), `log_23` @+9 s (Appendix A). This does **not**
> invalidate the architecture; it makes the reset-compensation path **mandatory**, not
> optional. The two mechanisms must both be handled (and not double-counted, §5.2):
> `world→map` for origin-reference (`ref_*`) placement, `map→odom` for the EKF local-estimate
> jumps (`delta_*`). The on-ground/boot origin establishment is the *largest* single jump, but
> it is not the only one.

What *is* ruled out:
- **ROS conversion / passthrough / msg version** — `hardware_abstraction` NED→ENU is faithful
  (`px4_hardware_abstraction.cpp:266`); `estimation_manager` is passthrough
  (`px4_passthrough_estimator`); TUI shows `estimated_state.z` directly (`tui_node.cpp:613`);
  `px4_msgs` is byte-compatible with firmware (`VehicleOdometry MESSAGE_VERSION = 0`).
- **`home_position` flicker** — commander-side (`HomePosition.cpp:369-404`, `:395-396`),
  calls no EKF reset, touches neither odom/lpos Z nor any counter. `home_position` is mutable
  by design and must **never** anchor a ROS frame.

---

## 4. How PX4 itself stays correct (the design we copy)
PX4's AUTO modes never store a fixed local-z setpoint. They keep the goal in **lat/lon/AMSL**
and re-project to local NED **every control cycle** with the *live* origin:

```
FlightTaskAuto.cpp:394   tmp_target(2) = -(setpoint.current.alt - _reference_altitude);
FlightTaskAuto.cpp:566   _reference_altitude = vehicle_local_position.ref_alt;   // live, each cycle
mission_block.cpp:867    relative-alt mission/takeoff item -> AMSL via home.alt
```

When the origin re-anchors, `ref_alt` changes; on that same cycle the **estimate**
`-(alt − ref_alt)` and the **setpoint** `-(target_alt − ref_alt)` shift by the *same* delta,
so tracking error stays ~0 and the vehicle doesn't move. PX4 tolerates an arbitrary, drifting
local frame because it **never caches a local setpoint** — it anchors goals to a stable datum
(home.alt / AMSL) and re-projects with the live origin.

**In offboard, PX4 receives our `trajectory_setpoint` already in local NED and trusts it — so
that global→local projection is *our* responsibility.** We're currently not doing it. This
plan makes us do it.

---

## 5. Frame architecture

### 5.1 Why the current frames are wrong (precise statement)
Our "world" frame is bolted to PX4's local frame by a **static** offset, but the local origin
**moves**. With static `world→odom` offset `O`:

```
world_z(P) = -(alt_P − ref_alt) + O      →  changes when ref_alt changes
```

so a **stationary** point's world coordinate drifts. A correct world frame anchors to a
**physical** altitude and uses a **dynamic** transform:

```
world_z(P) = alt_P − world_datum_alt                       (fixed; no ref_alt)
local_z    = -(world_datum_alt + world_z − ref_alt)        (recomputed each cycle, live ref_alt)
```

Now `ref_alt` changes are absorbed by the transform and `world_z(P)` is invariant. (Velocity
is translation-invariant; only position/Z holds were ever affected.)

### 5.2 Frame tree and ownership
```
world ──(1)──► map ──(2)──► odom ──(3)──► base_link ──(4)──► base_link_frd
```

| Frame | Meaning | Convention | **Placed by** | Namespace (§5.5) |
|---|---|---|---|---|
| `world` | shared, fixed, physically-anchored fleet frame | ENU | **us** (configured datum) | **shared** (unprefixed) |
| `map` | per-UAV non-drifting frame; origin = EKF local origin in world | ENU | **PX4** (`ref_*`) | per-UAV `<ns>/map` |
| `odom` | raw PX4 local estimate frame; origin wanders on resets | ENU | **PX4** (`vehicle_local_position`) | per-UAV `<ns>/odom` |
| `base_link` | vehicle body | FLU | **PX4** (estimate) | per-UAV `<ns>/base_link` |
| `base_link_frd` | body, PX4 convention | FRD | static | per-UAV `<ns>/base_link_frd` |

| # | Edge | Source (PX4 `vehicle_local_position` unless noted) | Updates |
|---|------|-----------------------------------------------------|---------|
| 1 | `world→map` | `geodeticToEnu(world_datum, ref_lat, ref_lon, ref_alt)`, gated `xy_global && z_global`; vertical from the ground datum when GPS-denied (§5.4) | **set once at first valid `ref_*`; NOT refreshed on later `ref_timestamp`** (see double-count note) |
| 2 | `map→odom` | accumulated **negative** **position** reset offset: `−Σ nedToEnu(delta_xy, delta_z)` (the **inverse** of the PX4 estimate jump). **NO yaw** — see heading note. | **steps only on an `xy/z_reset_counter` change**; else constant |

> **CORRECTION (heading is NOT compensated).** Earlier drafts accumulated `−Σ delta_heading`
> into a `map_to_odom_yaw`. That is **wrong** and was removed. `world` and `odom` are both
> north-aligned ENU, so the transform between them is a **pure translation** — there is no yaw.
> A heading reset is an EKF **correction of the orientation estimate** (expressed identically in
> both frames), not a frame-origin change, so the corrected orientation must **pass through**
> unchanged. Compensating it makes the published world heading wrong by the accumulated amount,
> which corrupts any velocity rotated by it (it broke inter-UAV BVC in SITL — the coordinator
> rotates body velocity by this orientation). `map_to_odom_yaw` is kept in `FrameAnchor` for
> message stability but is always 0.

**Sign (TF map→odom).** For parent `map`, child `odom`: `p_map = t_map_odom + p_odom`. If raw
PX4 pose jumps `p_odom_new = p_odom_old + delta` while the vehicle is physically still, then
to hold `p_map` constant: `t_new = t_old − delta`. So accumulate the **inverse**:
`map_to_odom_enu −= nedToEnu(delta_ned)`, `map_to_odom_yaw −= delta_heading_enu`. (`+Σdelta`
would be correct only for the opposite edge `odom→map`.) `delta_*` is PX4's published local
jump — horizontal `ekf_helper.cpp:121`, vertical `position_fusion.cpp:195`, and the same value
PX4 adds to stale setpoints in `MulticopterPositionControl.cpp:703`. **Lock the sign with the
§9 synthetic test**, do not trust this prose.

**No double-counting.** A PX4 **origin move** is reported as BOTH a `ref_*` change AND a
coincident `*_reset_counter`/`delta_*` (origin set bumps the counter: `ekf_helper.cpp:121,145`).
So the two edges are *not* independently separable — if `world→map` chases `ref_*` live *and*
`map→odom` accumulates the coincident delta, the origin move is compensated twice. Resolution
(simplest, REP-105-aligned): **`map→odom` absorbs ALL deltas; `world→map` is the one-time
global anchor** from the first valid `ref_*` (+ datum) and is not refreshed afterward —
subsequent origin moves already live in the `map→odom` deltas. (Alternative: track `ref_*` live
in `world→map` and exclude origin-coincident deltas from `map→odom` by detecting the
`ref_timestamp` change — more code, same result. Pick one; default to the former.)
| 3 | `odom→base_link` | raw PX4 local estimate (NED→ENU). **Untouched.** | every sample |
| 4 | `base_link→base_link_frd` | static FLU↔FRD | static |

**Field→edge split (critical):** `ref_*` feeds **`world→map`** (global placement); the reset
deltas feed **`map→odom`** (local correction). Different edges, different fields of the same
message. The EKF origin *is* the odom origin, so using `ref_*` for `map→odom` would collapse
it to identity and lose reset handling.

### 5.3 We command in WORLD; generators are NOT rewritten
All of `world/map/odom` are Cartesian ENU — they differ only by a rigid transform (translation
+ yaw). Trajectory generators' math (lines, holds, climbs) is **frame-agnostic**. So:
- generators, `control_manager`: **unchanged** (just relabel `frame_id` to `world`).
- the only real change is at the **`hardware_abstraction` boundary**:
  - **estimate out:** publish state in `world` (apply `odom→world` once).
  - **setpoint in:** convert each generator setpoint `world→odom→NED` *every cycle* with the
    live transform, before sending `trajectory_setpoint`.
- A static `world` goal thereby becomes a *moving* odom/NED setpoint that tracks PX4's origin —
  generators never see a delta. Capturing `startState` from the *world* estimate (non-drifting)
  is automatically stable. `TakeoffGenerator`'s absolute-Z becomes correct *for free* because
  `world z = 0` is now the ground/datum.

### 5.4 Vertical ground datum (what makes "2 m" mean 2 m above ground)
`world`'s Z must anchor to a **stable physical altitude**:
- **GPS (`z_global == true`):** AMSL. `world_datum_alt` is an AMSL number; `ref_alt` is AMSL;
  §5.1 math holds directly.
- **GPS-denied (`z_global == false`):** `ref_alt` is NaN — no AMSL. Anchor `world` Z to a
  **ground datum**: the estimate Z captured at arm/first-valid (vehicle on the ground), or a
  downward range finder's `dist_bottom` (true AGL). `world == map` horizontally in this case.
- **Never** use `home.z` (mutable).

### 5.5 Multi-UAV namespacing (REQUIRED)
This is a fleet system, so frame IDs and topics must be namespaced consistently. TF frame
IDs are global strings in one tree — they **must** be unique per UAV, and the shared frame
**must** be identical across UAVs.

- **`world` — SHARED, NOT namespaced.** Exactly the same frame_id string (e.g. `world`) for
  every UAV. It is the single common TF root and the only frame in which cross-UAV poses may
  be compared. As the root it has no parent transform to publish; each UAV publishes its own
  `world→<ns>/map` edge into it.
- **`map`, `odom`, `base_link`, `base_link_frd` — PER-UAV, namespaced** with the UAV prefix,
  e.g. `uav1/map`, `uav1/odom`, `uav1/base_link`. **Change from current code:** today `map`
  (and `world`) are published unprefixed while only `odom`/`base_link` carry the prefix
  (`frame_transformer.cpp` `worldFrame_`/`mapFrame_` vs `composeFrame(framePrefix_, ...)`).
  Under this model `map` is per-UAV (each UAV's EKF origin placed in world) and **must** move
  to the namespaced form; `world` stays the shared, unprefixed string.
- **Fleet TF tree:** one `world` → for each UAV `world→<ns>/map → <ns>/odom → <ns>/base_link
  → <ns>/base_link_frd`. Each `world→<ns>/map` is distinct (its own `ref_*`). Exactly one
  publisher per `world→<ns>/map` edge (the UAV's own `frame_transforms` node) — never two
  UAVs publishing the same unprefixed `map`.
- **`world_datum_*` must be IDENTICAL across the fleet** (shared config), or the UAVs place
  themselves in inconsistent worlds and relative positions are wrong. Single shared datum;
  per-UAV fallback to first `ref_*` is only valid in single-agent mode.
- **Topics** are already per-UAV via node namespace (`/<ns>/...`): publish `FrameAnchor`,
  `estimated_state`, `trajectory_setpoint`, etc. under each UAV's namespace. The
  `multi_agent_coordinator` / fleet bus compares poses **only** in the shared `world` frame —
  never across two `<ns>/odom` or `<ns>/map` frames (RULE: go up to `world` to compare).
- The `frame_prefix` param drives all per-UAV frame strings; `world_frame` and
  `world_datum_*` are fleet-wide. Keep this split explicit in params/bringup.

---

## 6. Implementation plan (per package)

### 6.1 `peregrine_interfaces` — new message
`src/peregrine_core/peregrine_interfaces/msg/FrameAnchor.msg` (+ `CMakeLists.txt`):
```
std_msgs/Header header
geometry_msgs/Vector3 map_to_odom_enu   # −Σ nedToEnu(delta_xy, delta_z); INVERSE of PX4 jump; steps on reset
float64 map_to_odom_yaw                  # always 0 (heading is NOT compensated; see §5.2 correction)
float64 ref_lat
float64 ref_lon
float64 ref_alt                          # AMSL
bool    ref_valid                        # xy_global && z_global
uint64  ref_timestamp
float64 ground_datum_z                   # estimator-frame Z taken as ground (capture-at-arm / dist_bottom)
bool    ground_datum_valid
```

### 6.2 `hardware_abstraction` — state source, anchor, boundary conversion
`.../hardware_abstraction/src/px4_hardware_abstraction.cpp` + header. This node is the single
owner of the `world↔PX4-local` math (it already does ENU↔NED here).

**State source — keep `VehicleOdometry`, ADD `VehicleLocalPosition` for metadata only
(recommended; less invasive).** The raw pose/orientation/rates path (`VehicleOdometry`, with
its already-tested velocity-frame handling and NED→ENU conversion) drives `odom→base_link`
**unchanged**. Add a `VehicleLocalPosition` subscription used **only** for `ref_lat/lon/alt`,
`xy_global`/`z_global`, the `*_reset_counter`s and `delta_*`. Both come from the same EKF
`getPosition()` cycle, so the lpos deltas correspond to the jumps in the odom pose.
- *Caveat:* the two topics arrive in separate callbacks, so at a reset instant there can be a
  one-sample skew between the odom jump and the offset update → at most one sample of transient
  error, self-correcting. Acceptable. (Full alternative: switch the state source entirely to
  `VehicleLocalPosition` + `VehicleAttitude` + `VehicleAngularVelocity` — more invasive, no real
  benefit here. Only do it if you want a single source of truth.)
- velocity (if you ever do the full switch): lpos `v*` is **NED world** →
  `v_body_frd = q_ned_frd^{-1} v_ned`, `v_body_flu = frdToFlu(...)`; covariance from `eph/epv`.

**Reset accumulator (INVERSE sign — see §5.2), gated on counter change (NOT `delta != 0`** —
`delta_*` is last-reset-only, not cumulative):
  - `xy_reset_counter` ↑ → `map_to_odom_enu.xy −= nedToEnu(delta_xy)`
  - `z_reset_counter` ↑ → `map_to_odom_enu.z −= nedToEnu(delta_z)`
  - `heading_reset_counter` ↑ → **track the counter only; do NOT accumulate yaw** (heading is
    an estimate correction, not a frame change — §5.2 correction). Orientation passes through.
  - this offset absorbs **all** deltas (incl. origin moves); `world→map` is therefore the
    one-time anchor and must NOT also react to `ref_*` (§5.2 double-count note).
  - re-baseline if a counter jumps backward (EKF reinit). Do **not** re-baseline merely because
    `ref_timestamp` changed — that is an origin move whose delta you are already accumulating.
- Capture the **ground datum** (estimate Z at arm/first-valid, or `dist_bottom`).
- Read `world_datum_*` params. Compose `odom→world` = (`map→odom` from offset) ∘ (`world→map`
  set **once** from the first valid `ref_*` + datum, or ground datum when GPS-denied; NOT
  refreshed on later `ref_timestamp` — §5.2).
- **Publish the estimate in `world`.** Keep the raw odom internally for `odom→base_link`/TF.
- **Convert incoming setpoints `world→odom→NED` each cycle** (position **and** yaw; velocity/
  accel are translation-invariant — rotate only, no offset). Then send `trajectory_setpoint`.
- Publish `FrameAnchor` (reliable, low-rate/on-change).
- Keep `nowMicros()` timestamps; do **not** disable timesync. PX4's own reset net stays on;
  residual double-correction is one bridge-latency window, self-correcting, never cumulative.

### 6.3 `frame_transforms` — TF tree mirror
`.../frame_transforms/src/frame_transformer.{cpp,hpp}` + `frame_transforms_parameters.yaml`.
- **Remove** the GNSS/`home_lat/lon` init (`frame_transformer.cpp:240-310`, incl. the `alt=0`
  `geodeticToEnu` at `:287`, and `onGnss`/`onGpsStatus`/`tryInitHome`).
- Subscribe to `FrameAnchor`; publish TF: `world→<ns>/map` (from first valid `ref_*`+datum,
  set once — §5.2), `<ns>/map→
  <ns>/odom` (dynamic, steps on reset), `<ns>/odom→<ns>/base_link` (estimate),
  `<ns>/base_link→<ns>/base_link_frd` (static). This is a **visualization/multi-agent
  mirror** — no control role.
- **Namespacing (§5.5):** prefix `map`/`odom`/`base_link`/`base_link_frd` with the UAV
  `frame_prefix`; keep `world` the shared unprefixed string. This moves `map` from the
  current unprefixed form to per-UAV — update `mapFrame_` to use `composeFrame(framePrefix_,
  ...)` like the others, and leave `worldFrame_` shared.
- Params: drop `home_*`, `gps.*`; add fleet-wide `world_datum_lat/lon/alt` (identical across
  UAVs) and keep per-UAV `frame_prefix`/`world_frame`.
- Add `geodeticToEnuAzEq()` to `conversions.hpp` (PX4 azimuthal-equidistant, inline, no extra
  dep) so `world↔ref` matches the EKF's `MapProjection`; unit-test vs PX4 reference values.

### 6.4 `trajectory_manager` — relabel only
`.../trajectory_manager/src/...`. Generators **unchanged**. Set `frame_id = world` on captures
and setpoints. Re-check the configured takeoff target value now that it means *world Z above
datum* (e.g. `2.0` = 2 m above ground). No relative-height hack needed.

### 6.5 Untouched
`estimation_manager` (passthrough), `control_manager` (passthrough), `tui_status`,
`multi_agent_coordinator` (now consumes a *correct* shared world frame). The payoff of
concentrating all corrections at the `hardware_abstraction` boundary + TF mirror.

---

## 7. Open decisions
1. **Ground datum source:** capture-at-arm (default; no extra HW) vs range finder `dist_bottom`
   (true AGL; needs sensor) vs configured.
2. **Where the `world↔odom` composition lives:** in `hardware_abstraction` (default; one place,
   one cycle) vs a dedicated localization node. Default keeps the change small.
3. **`world_datum`:** configured site origin (`world_datum_lat/lon/alt`, shared per fleet),
   falling back to first valid `ref_*` for single-agent (`world == map`).

## 8. Sequencing (foundation-first — do NOT start with a takeoff patch)
1. `FrameAnchor` msg (§6.1) + `hardware_abstraction` state source & reset accumulator & ground
   datum & boundary conversion (§6.2). This is the foundation; once the estimate is in `world`
   and setpoints reproject per cycle, **all** Z consumers (takeoff included) become correct.
2. `frame_transforms` TF mirror (§6.3) — enables multi-agent + correct RViz/world.
3. `trajectory_manager` relabel + re-check takeoff value (§6.4).
4. Multi-UAV `world_datum` validation.

A standalone "make takeoff relative" patch is explicitly **not** step 1: it's a throwaway that
fixes one consumer, gets superseded by step 1, and de-risks none of the design. Only do it if
flight is needed before step 1 lands.

## 9. Tests
- **Unit — sign lock (DO THIS FIRST, §5.2):** synthetic **stationary** vehicle. Feed a raw PX4
  pose jump of `+delta` (with a `*_reset_counter` increment and the matching `delta_*`). Assert
  `map`/`world → base_link` is **unchanged** across the jump (i.e. `map_to_odom −= delta`
  cancels it). A wrong sign doubles the jump instead of cancelling — this test catches it.
- **Unit (`hardware_abstraction`):** synthetic lpos sequences (single/multiple/simultaneous
  xy+z, heading, counter wrap, origin-move = `ref_timestamp` + coincident delta) ⇒ world
  estimate continuous; offset = `−Σδ`; **origin move compensated exactly once** (no
  double-count between `world→map` and `map→odom`); ground datum captured once and stable;
  world↔NED setpoint round-trip (incl. yaw).
- **Unit (`frame_transforms`):** `FrameAnchor` → expected `world→map` (AMSL Z; ref change;
  datum-from-first-ref) and `map→odom` (steps on reset); `geodeticToEnuAzEq` vs PX4 reference.
- **SITL:** offboard hold + induced resets (Z: `EKF2_HGT_REF` toggle; XY: `EKF2_GPS_CTRL`) ⇒
  counter ticks, world estimate continuous, Gazebo ground-truth unmoved. Takeoff from a field
  whose EKF origin is offset from ground ⇒ climbs the commanded height **above ground**.
- **SITL multi-UAV:** two agents at known offset; B-relative-to-A in `world` matches
  ground-truth within GPS tol; Z agreement via AMSL.
- **`view_frames`:** `world→map→odom→base_link`; `map→odom` steps on induced resets.

---

## Appendix A — flight-log evidence
Tooling: `pyulog` over `px4_logs_peregrine/` and `peregrine_px4_logs_2/`.
- **Every log starts at arm** (`pre-ground span ≈ −0.0..−0.3 s`) → the boot/ground origin
  establishment is never captured, which is why armed-window-only analysis missed the cause
  and (wrongly) concluded "no resets."
- **Per-flight Z origin is arbitrary** (armed window, nominally identical hovers): `log_41 ≈
  −15 m` throughout, `log_30 ≈ +11.7 m`, `log_31 −5..+12`, `log_46 −3..+9.7`.
- **`z_reset_counter` climbs within a boot session:** 6, 8, 12, 45 (`log_38`).
- **In-flight resets DO occur** (`xy/z/heading_reset_counter` increment while armed):
  `log_42` @332.3 s (arm@329; xy 23→25, z 12→13, hdg 8→9), `log_44` @1198.9 s (arm@1067.9;
  xy 21→22, z 11→12, hdg 10→12) and @1081.9 s (hdg 9→10), `log_23` @373.2/380.2 s
  (xy 11→13, z 7→8). So both the on-ground origin establishment **and** in-flight resets
  happen — reset compensation is mandatory. (`log_6/7` happened to be reset-free while armed,
  which earlier misled the analysis into "no in-flight resets.")
- Large Z excursions seen *during AUTO_LAND* (e.g. `+6.8 m` in `log_7`) are the land-detector /
  stale-setpoint issue (`stale-setpoint-after-managed-land.md`), a separate problem.

## Appendix B — source references
PX4 (`/scratch/robotics/quad-firmware/PX4-Autopilot`):
- `getPosition()` Z: `src/modules/ekf2/EKF/estimator_interface.cpp:628`; origin-or-0
  `.../estimator_interface.h:328`
- origin alt from baro: `.../aid_sources/barometer/baro_height_control.cpp:170`;
  `.../ekf_helper.cpp:219-241`
- z-reset writers (all EKF): `.../position_fusion.cpp:185,222`; callers `ekf_helper.cpp:140,145,238`,
  `baro_height_control.cpp:137`, `gnss_height_control.cpp:114,157`, `range_height_control.cpp`,
  `ev_height_control.cpp`, `fake_height_control.cpp`
- lpos `ref_*`/`*_global`/`delta_*`/counters: `src/modules/ekf2/EKF2.cpp:1728-1781`; odometry ==
  lpos position `EKF2.cpp:1700-1702,1823`
- navigator-style global→local projection: `.../flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:394,566`;
  `.../navigator/mission_block.cpp:189,867`
- home alt correction (off the odom path): `.../commander/HomePosition.cpp:369-404`

Peregrine (`/scratch/robotics/peregrine`):
- consumes `VehicleOdometry`, faithful NED→ENU: `.../hardware_abstraction/src/px4_hardware_abstraction.cpp:175,266`
- estimation passthrough: `.../estimation_manager/src/px4_passthrough_estimator.cpp`
- TUI shows `estimated_state.z`: `.../tui_status/src/tui_node.cpp:613`
- GNSS/home frame hack to remove: `.../frame_transforms/src/frame_transformer.cpp:240-310` (`alt=0` `:287`)
- takeoff absolute-Z assumption: `.../trajectory_manager/src/generators.cpp` (TakeoffGenerator),
  `.../trajectory_manager/src/trajectory_manager_node.cpp:627`
