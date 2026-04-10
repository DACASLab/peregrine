# PEREGRINE Core Example Validation

Date: 2026-04-05

Scope:
- Validated the `hardware_abstraction_example` launches in `src/peregrine_core/examples/hardware_abstraction_example/launch`
- Excluded `example12_safety_validation.launch.py` per request
- Used the docker `make` workflows from `docker/Makefile`
- Checked GCS and TUI data flow
- Checked RViz where applicable

## Build and Environment

Host-side build commands used:

```bash
cd /scratch/robotics/peregrine/docker
make build-sim
make build-gcs
```

Important one-time fix applied before testing:

```bash
# Inside sim container
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --packages-select control_manager --cmake-clean-cache --event-handlers console_direct+
```

Reason:
- `example10_circle_figure8_demo.launch.py` initially failed because `control_manager` was still linked against a stale acados/NMPC artifact.
- After rebuilding `control_manager`, the non-SE3 examples ran correctly.

Loopback multicast check:

```bash
ip link show lo
```

Observed:
- Host loopback already had `MULTICAST` enabled.
- DDS loopback multicast was not the blocker for RViz in this session.
- RViz failures were X11/GLX context creation failures.

## Launch Ownership Check

This was re-checked directly from the launch files.

### Example launches that **do start their own PX4/Gazebo stack**

- `example10_se3_circle_figure8_demo.launch.py`
- `example10_se3_step_response_demo.launch.py`

Why:
- Both contain an explicit `ExecuteProcess(...)` that runs:

```bash
cd /opt/PX4-Autopilot && ... make px4_sitl gz_x500
```

### Example launches that **do not start PX4/Gazebo**

- `example8_px4_sitl_single_uav.launch.py`
- `example10_circle_figure8_demo.launch.py`
- `example11_multi_cycle_demo.launch.py`
- `example13_monitoring_demo.launch.py`
- `example14_multi_uav_circle_figure8.launch.py`
- `example15_multi_uav_circle_figure8_9uav.launch.py`
- `example16_controller_switch_demo.launch.py`
- `example17_controller_switch_inflight_demo.launch.py`
- `peregrine_single_container.launch.py`

Notes:
- `example8_px4_sitl_single_uav.launch.py` is named as if it brings up SITL, but it only starts:
  - `MicroXRCEAgent`
  - `PX4HardwareAbstraction`
  - `FrameTransformer`
- `example10`, `example11`, `example13`, `example16`, and `example17` all layer on top of `example8`, so they also expect PX4 SITL to already be running.
- `example14` and `example15` are only mission-client launch bundles for a multi-container stack that is already running.

### Safety example ownership

Not part of the requested run, but checked for completeness:

- `example12_safety_validation.launch.py` does not start PX4/Gazebo itself.
- It wraps `peregrine_single_container.launch.py`, which starts the stack side only.

## Simpler Bringup Paths

### Simpler single-UAV bringup already exists

Instead of manually using one terminal for PX4 and one for the example stack, there is already a single bringup wrapper:

```bash
ros2 launch peregrine_bringup single_uav_sitl.launch.py
```

What it does:
- optional cleanup
- starts PX4 SITL
- starts the single-UAV stack via `single_uav.launch.py`

This is the simpler bringup path if the goal is full single-UAV SITL + stack, not specifically exercising the example package launch files.

### Simpler multi-UAV SITL bringup already exists

There is already a dedicated multi-SITL bringup launch:

```bash
ros2 launch peregrine_bringup multi_uav_sitl.launch.py num_uavs:=N
```

What it does:
- starts shared Gazebo
- starts multiple PX4 SITL instances
- starts one MicroXRCE agent per PX4 instance

Important limitation:
- `multi_uav_sitl.launch.py` only brings up the PX4/Gazebo side.
- It does not launch the per-UAV PEREGRINE stacks.

### What `make multi-sitl` already does

The docker workflow is already using the bringup files under the hood:

- sim container:
  - runs `ros2 launch peregrine_bringup multi_uav_sitl.launch.py ...`
- each `uavN` container:
  - runs `ros2 launch peregrine_bringup single_uav.launch.py ...`

So the simpler multi-UAV bringup path already exists and is already wired into the compose flow.

### What still does not have a simpler wrapper

The mission demos themselves are still example-package launches:

- `example14_multi_uav_circle_figure8.launch.py`
- `example15_multi_uav_circle_figure8_9uav.launch.py`

There is not currently a separate `peregrine_bringup` mission wrapper for the multi-UAV demo path analogous to:

```bash
ros2 launch peregrine_bringup mission_circle_figure8.launch.py
```

That bringup mission wrapper exists only for the single-UAV hardware path.

## Commands Used

## Single-UAV external PX4 workflow

Used for:
- `example8`
- `example10`
- `example11`
- `example13`
- `example16`
- `example17`

Terminal 1 on host:

```bash
cd /scratch/robotics/peregrine/docker
make shell-sim
```

Terminal 1 inside container:

```bash
cd /opt/PX4-Autopilot
ROS_DOMAIN_ID=42 HEADLESS=1 make px4_sitl gz_x500
```

Terminal 2 on host:

```bash
cd /scratch/robotics/peregrine/docker
make shell-sim
```

Terminal 2 inside container:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch hardware_abstraction_example <launch-file>
```

## Self-contained SE3 workflow

Used for:
- `example10_se3_circle_figure8_demo.launch.py`
- `example10_se3_step_response_demo.launch.py`

Terminal 1 on host:

```bash
cd /scratch/robotics/peregrine/docker
make shell-sim
```

Terminal 1 inside container:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch hardware_abstraction_example <se3-launch-file>
```

## Multi-UAV + GCS workflow

Terminal 1 on host:

```bash
cd /scratch/robotics/peregrine/docker
make multi-sitl
```

Terminal 2 on host:

```bash
cd /scratch/robotics/peregrine/docker
make shell-gcs
```

Terminal 2 inside GCS container:

```bash
tmux ls
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=99
ros2 topic list | sort
```

Optional GCS tmux attach from host:

```bash
cd /scratch/robotics/peregrine/docker
make tmux-gcs
```

Terminal 3 on host:

```bash
cd /scratch/robotics/peregrine/docker
make multi-dev
```

Terminal 3 inside dev container:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch hardware_abstraction_example <multi-uav-launch-file>
```

## Results

| Launch | Result | Notes |
|---|---|---|
| `example8_px4_sitl_single_uav.launch.py` | PASS | Stack-only sanity check passed |
| `example10_circle_figure8_demo.launch.py` | PASS | Passed after rebuilding stale `control_manager` |
| `example10_se3_circle_figure8_demo.launch.py` | FAIL | SE3 takeoff stalled in `TAKING_OFF`, then `TRAJECTORY_EXECUTE_RESULT_TIMEOUT` |
| `example10_se3_step_response_demo.launch.py` | FAIL | Same SE3 takeoff timeout pattern |
| `example11_multi_cycle_demo.launch.py` | PASS | Full four-cycle mission completed |
| `example13_monitoring_demo.launch.py` | PASS with caveats | Mission path passed, TUI node alive, flight visualizer alive, RViz failed |
| `example14_multi_uav_circle_figure8.launch.py` | PASS | Default 2-UAV mission-client launch completed against running multi-container backend |
| `example15_multi_uav_circle_figure8_9uav.launch.py` | FAIL by default in `make multi-dev` shell | Launch defaulted to 9 clients, while backend had 6 UAVs; UAV1-6 passed, UAV7-9 timed out |
| `example15_multi_uav_circle_figure8_9uav.launch.py num_uavs:=6` | PASS | All 6 vehicles completed successfully |
| `example16_controller_switch_demo.launch.py` | FAIL | Passthrough cycle passed, switch to SE3 succeeded, SE3 takeoff timed out |
| `example17_controller_switch_inflight_demo.launch.py` | PASS | In-flight lifecycle controller switching completed successfully |

## Detailed Notes

### `example8_px4_sitl_single_uav.launch.py`

Observed:
- `microxrce_agent` started
- `bridge_container` started
- `px4_hardware_abstraction` started
- `frame_transformer` initialized home GPS origin
- `/odometry` and related topics were live

### `example10_circle_figure8_demo.launch.py`

Observed:
- `control_manager` configured with `control_manager::Px4PassthroughController`
- mission completed:
  - takeoff
  - circle
  - figure-8
  - land

### `example11_multi_cycle_demo.launch.py`

Observed:
- full configured sequence completed:
  - `circle`
  - `figure8`
  - `circle`
  - `figure8`

### `example13_monitoring_demo.launch.py`

Launch used:

```bash
ros2 launch hardware_abstraction_example example13_monitoring_demo.launch.py start_tui:=true start_rviz:=true
```

Observed:
- mission completed successfully
- `tui_status_node` was running and subscribed to the expected live status topics
- `flight_visualizer_node` was running and publishing:
  - `/viz/actual_path`
  - `/viz/reference_path`
  - `/viz/markers`
- `/estimated_state` was live at about `250 Hz`
- `/viz/markers` was live at about `15 Hz`

Caveats:
- local terminal size was too small for ncurses to render the full TUI layout
- RViz failed with GLX/X11 window creation errors

### `example16_controller_switch_demo.launch.py`

Observed:
- first passthrough-controlled cycle completed
- lifecycle switch to `control_manager::Se3Controller` succeeded
- `control_manager` remained active and publishing `/control_output`
- UAV remained `ARMED`, `OFFBOARD`, `detail=TAKEOFF_FAILED`
- second-cycle SE3 takeoff failed with:
  - `TRAJECTORY_EXECUTE_RESULT_TIMEOUT`

This looks like a real regression in the SE3 takeoff path, not a launch misuse issue.

### `example17_controller_switch_inflight_demo.launch.py`

Observed:
- takeoff under passthrough succeeded
- switch to SE3 while airborne succeeded
- circle under SE3 succeeded
- switch back to passthrough succeeded
- figure-8 succeeded
- switch back to SE3 succeeded
- final circle and land succeeded

This narrows the SE3 issue:
- in-flight switching works
- starting a fresh takeoff under SE3 is what fails

### `example10_se3_circle_figure8_demo.launch.py`

Observed:
- launch brought up PX4 SITL, bridge stack, manager stack, and `Se3Controller`
- readiness checks passed
- PX4 reported `Ready for takeoff!`
- UAV remained in `TAKING_OFF`
- mission failed with `TRAJECTORY_EXECUTE_RESULT_TIMEOUT`

### `example10_se3_step_response_demo.launch.py`

Observed:
- same startup path as the SE3 circle/figure-8 demo
- same failure mode:
  - takeoff begins
  - state remains `TAKING_OFF`
  - mission times out

## GCS and TUI Findings

GCS bringup:
- container started successfully with `make shell-gcs`
- tmux session `gcs` started
- domain `99` saw namespaced topics for all running UAV stacks

Observed on GCS:
- `/uav1` through `/uav6` topics were present
- `/uav1/estimated_state` was active around `180-218 Hz` during startup
- `/uav3/uav_state` showed live flight state during mission execution

TUI findings:
- data flow was correct
- stock tiled GCS panes were too small in this terminal geometry for `tui_status` to render fully
- this was a layout/terminal-size issue, not missing topic data

## RViz Findings

Single-UAV and GCS RViz attempts both failed.

Observed failures:
- `Unable to create glx visual`
- `Invalid parentWindowHandle`
- `Unable to create a suitable GLXContext`
- `Unable to create the rendering window after 100 tries`

Conclusion:
- multicast loopback was already correct
- RViz failures were due to the X11/GLX environment in this session

## Multi-UAV Default Mismatch in `example15`

Important finding:
- `make multi-sitl` regenerated compose for `NUM_UAVS=6` from `docker/.env.local`
- inside the `make multi-dev` shell, `NUM_UAVS` and `GCS_NUM_UAVS` were unset
- therefore `example15_multi_uav_circle_figure8_9uav.launch.py` fell back to its internal default of `9`

Observed directly in the dev shell:

```bash
echo NUM_UAVS=$NUM_UAVS GCS_NUM_UAVS=$GCS_NUM_UAVS
```

Output:

```bash
NUM_UAVS= GCS_NUM_UAVS=
```

Effect:
- `example15` defaulted to 9 mission clients
- only UAV1-UAV6 had a running backend
- UAV7-UAV9 failed preflight with no `uav_state`

Workaround that passed:

```bash
ros2 launch hardware_abstraction_example example15_multi_uav_circle_figure8_9uav.launch.py num_uavs:=6
```

## Recommended Interpretation

- The non-SE3 single-UAV examples are in good shape after fixing the stale `control_manager` build.
- The SE3 takeoff path is currently broken in multiple places:
  - `example10_se3_circle_figure8_demo`
  - `example10_se3_step_response_demo`
  - `example16_controller_switch_demo`
- In-flight controller switching itself is working, based on `example17`.
- The multi-UAV backend and GCS bridge work.
- `example15` has a practical usability bug because its default `num_uavs` does not match the docker `make multi-dev` shell environment.

