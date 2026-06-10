# BVC mission SITL harness (`sitl_bvc_mission.py`)

End-to-end check that inter-UAV Buffered Voronoi Cell (BVC) avoidance works during
the **real surveillance mission**, dispatched through the Behavior Trees.

It triggers one `ExecuteTree` goal per UAV (the same `MultiUavSurveillanceUav<N>`
trees the mission uses), watches the shared `/fleet/agent_state` bus, and asserts
that any two airborne, same-altitude UAVs stay at least `2*safety_radius` apart.

## Run (docker multi-agent sim)

```bash
# Terminal 1 — Gazebo + 2 UAV stacks with BVC enabled
cd docker
NUM_UAVS=2 ENABLE_AVOIDANCE=true make multi-sitl

# Terminal 2 — GCS (domain 99, bridges execute_tree + /fleet/agent_state)
cd docker
make shell-gcs
# inside the gcs container:
python3 /ros2_ws/tools/sitl_bvc_mission.py --uavs uav1,uav2 --safety-radius 2.5
```

Artifacts land in `artifacts/bvc_mission/` (`separation.csv`, `summary.json`).
Exit code 0 = PASS (all trees SUCCEEDED **and** separation held).

## Baseline (prove it's doing something)

Bring the sim up with `ENABLE_AVOIDANCE=false` and re-run the harness: the same
same-cell sweep should drive separation below `2*safety_radius` (FAIL), confirming
the avoidance layer is what keeps them apart when enabled.

## Useful flags

| flag | meaning |
|---|---|
| `--uavs uav1,uav2` | UAV namespaces to command/monitor |
| `--trees A,B` | override tree ids (default `MultiUavSurveillanceUav<N>`) |
| `--safety-radius 2.5` | per-agent `r_s`; required separation defaults to `2*r_s` |
| `--min-sep` / `--tol` | override required separation / tolerance (m) |
| `--altitude-band 2.0` | `|dz|` within which a pair counts as same-altitude |
| `--no-trigger` | monitor only (mission triggered elsewhere) |
| `--mission-timeout 600` | abort + report after N seconds |

## Notes
- Must run in the GCS ROS domain (the `make shell-gcs` container is already domain
  99 with the Zenoh bridge). The GCS bridge was extended to import
  `/fleet/agent_state` for exactly this.
- Separation is checked **horizontally** (BVC is 2D) and only while both UAVs are
  above `--airborne-z` and within `--altitude-band` — i.e. when BVC is responsible.
