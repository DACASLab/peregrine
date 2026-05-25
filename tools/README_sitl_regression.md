# Headless SITL Regression Harness

`tools/sitl_regression.py` runs repeatable PX4/Gazebo/Peregrine regression
cases and grades them from ROS topic evidence rather than mission log strings.

## Common Commands

```bash
# Cheap listing / planning
tools/sitl_regression.py --list
tools/sitl_regression.py --suite bt --dry-run

# Rebuild the workspace, then run the BT examples
tools/sitl_regression.py --suite bt --build

# Run one case
tools/sitl_regression.py --case bt/takeoff_hover_land

# Run Python-client examples
tools/sitl_regression.py --suite python-client

# Run the 3-UAV headless multi-SITL mission
tools/sitl_regression.py --suite multi-uav --num-uavs 3
```

Artifacts are written under `artifacts/sitl/<timestamp>_<suite>/`:

- `summary.json`
- `summary.md`
- per-case PX4/core/mission/BT logs
- per-case topic observations

## Current Suites

- `bt`: all XML trees in `peregrine_bt/trees`
- `python-client`: representative hardware abstraction Python-client missions
- `multi-uav`: 3-UAV circle + figure-8 mission using the multi-container stack
- `smoke`: `bt/takeoff_hover_land` plus `python/circle_figure8`

## Environment Defaults

Single-UAV cases use:

- `ROS_DOMAIN_ID=42`
- `ROS_LOCALHOST_ONLY=1`
- `PX4_PARAM_UXRCE_DDS_PTCFG=1`
- `PX4_PARAM_UXRCE_DDS_DOM_ID=42`
- `HEADLESS=1`

Multi-UAV cases default to 3 UAVs on ROS domains `1..3`, namespaces
`/uav1..3`, XRCE ports `8888, 8890, 8892`, and headless Gazebo.

## Pass/Fail Evidence

Each case is graded with topic observations:

- `/state`: altitude and final height
- `/uav_state`: armed/offboard/readiness/state enum
- `/fmu/out/vehicle_status_v1`: PX4 nav and arming state
- `/safety_status`: nominal/warning/critical
- `/status`, `/gps_status`: PX4 connection, battery, GPS readiness

Logs are retained only to explain failures, for example BT XML load errors or
action timeouts.

Each case runs for its configured timeout, then the harness scores the topic
evidence, tears down PX4/Gazebo, and moves to the next case. The timeout is
chosen per example rather than inferred from log strings.
