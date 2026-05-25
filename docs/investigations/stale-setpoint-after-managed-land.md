# Stale Setpoint + FSM Divergence After Managed Land

**Date:** 2025-05-25
**Status:** Analysis complete, fixes pending

## Problem

After a managed takeoff → land → raw arm sequence (`T → L → A` via TUI), the
vehicle climbs back to its previous hover altitude on stale setpoints, and the
manager rejects subsequent land commands.

## Observed State After T → L → A

| Layer              | Expected        | Actual                            |
|--------------------|-----------------|-----------------------------------|
| uav_manager FSM    | Armed           | **Idle** (raw arm bypassed FSM)   |
| PX4 armed          | true            | true                              |
| PX4 mode           | safe / manual   | **OFFBOARD** (stale setpoint stream)|
| trajectory_manager | idle / ground   | **hold at z=5.0** (stale)         |
| control_manager    | not publishing  | **publishing stale setpoints**    |

## Hazard Chain

```
managed takeoff
  └─ arm → offboard → trajectory climb → hover at z=5.0
  └─ trajectory_manager: holdGenerator_ freezes at z=5.0
  └─ control_manager: computing + publishing offboard setpoints from hold reference

managed land
  └─ PX4 set to LAND mode → descends → auto-disarms
  └─ trajectory_manager: STILL publishing hold at z=5.0 (nobody told it to stop)
  └─ control_manager: STILL publishing offboard setpoints
  └─ PX4 ignores them (in LAND mode, then disarmed)
  └─ uav_manager FSM: Idle ✓, PX4 disarmed ✓ — looks clean

raw arm (TUI 'a' → /uavN/arm → hardware_abstraction)
  └─ PX4 re-arms
  └─ Stale setpoint stream at z=5.0 is already flowing
  └─ PX4 has valid offboard setpoint stream → enters OFFBOARD
  └─ Vehicle climbs to 5m
  └─ FSM still Idle → managed land rejected
```

## Root Cause: Two Independent Failures

### 1. Stale setpoint stream (safety issue)

`trajectory_manager` uses a `HoldPositionGenerator` as its default when no
trajectory goal is active. After takeoff completes, it freezes at the hover
position (z=5.0) and publishes indefinitely — through landing, disarm, and
re-arm. This is by design to keep PX4's offboard setpoint stream alive during
hover, but it persists when it shouldn't.

Source: `trajectory_manager_node.hpp:26-28` — "When no active goal is running,
a HoldPositionGenerator continuously publishes the last known position to keep
the PX4 offboard setpoint stream alive."

Landing is the **only** managed operation that hands control to PX4 (via
`callSetModeService("land")`). During takeoff, hover, and flight, the manager
owns offboard control and the setpoint stream is correct. The stale state only
becomes dangerous after the land→idle transition, because the pipeline was never
told the vehicle is on the ground.

### 2. FSM divergence (operational issue)

TUI arm/disarm (`'a'`/`'d'` keys) call `/uavN/arm` directly via
`hardware_abstraction`, not through `uav_manager`. This arms PX4 without moving
the supervisor FSM out of Idle. Same bypass can happen from RC transmitter,
QGroundControl, or any raw service call.

With FSM stuck in Idle, `uav_manager` rejects land goals (only accepted from
Armed, TakingOff, Hovering, or Flying — `uav_manager_node.cpp:790`).

## Proposed Fixes

Both fixes are needed. The setpoint fix prevents the hazard; the FSM fix
maintains operational correctness.

### Fix 1: Reset hold position on land (trajectory_manager)

**What:** `trajectory_manager` subscribes to the `uav_state` topic (already
published by `uav_manager`). When state transitions to `LANDED` or `IDLE`,
call `switchToHoldFromState()` with the current estimated state.

**Effect:** After managed land, `holdGenerator_` resets to ground-level position.
If PX4 re-arms and enters offboard, the setpoint says "hold at ground" — vehicle
stays put. The setpoint stream keeps flowing (harmless when PX4 isn't in
offboard), but its target is safe.

**Scope:** ~15-20 lines in `trajectory_manager_node.cpp` (subscription +
callback). No interface changes.

**Why trajectory_manager, not control_manager:** The stale reference originates
in `holdGenerator_`. control_manager correctly computes output from whatever
reference it receives — the reference itself is what's wrong.

### Fix 2: FSM reconciliation on PX4 status (uav_manager)

**What:** In `uav_manager`'s existing `onPx4Status` callback, detect arm/disarm
divergence and reconcile:

- PX4 armed + FSM in {Idle, Landed} → apply new `ExternalArmDetected` event → Armed
- PX4 disarmed + FSM in {Armed, Landed} → apply existing `DisarmCompleted` → Idle

**New FSM event:** `ExternalArmDetected` with `GuardId::Always` (unguarded).
Cannot reuse `ArmRequested` because it has the `TakeoffReady` guard, which
would reject reconciliation when health checks fail — but the FSM should
reflect physical reality regardless of health.

**Scope:**
- `supervisor_state_machine.hpp`: add `ExternalArmDetected` to enum
- `supervisor_state_machine.cpp`: add 2 transition rules + toString case
- `uav_manager_node.cpp`: ~10 lines in `onPx4Status` callback

**Guard against log spam:** Only attempt reconciliation when the specific
(FSM state, PX4 armed) mismatch exists, not on every status tick.

## What This Doesn't Cover

These are noted but not in scope for the immediate fix:

- **RC takeover (authority loss):** RC pilot switches out of offboard during
  flight. PX4 enters POSCTL/MANUAL, manager's setpoints are ignored. Once the
  RC pilot has control, the manager is no longer the authority — PX4 and the
  pilot own safety. FSM staleness is cosmetic at that point.

- **PX4 disarm during flight state:** Kill switch or crash while FSM is in
  Hovering/Flying/TakingOff. Complex to handle because active action goals
  need cancellation. Existing failsafe detection (`FailsafeDetected` →
  Emergency) covers the PX4-initiated cases.
