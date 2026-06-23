# GCS `/uav4/execute_tree` Hardware Debug Handover

Date: 2026-06-23

## Problem

Hardware flight on UAV4 works from the GCS TUI for arm, takeoff, and land, but the GCS command pane fails when running the surveillance trigger:

```bash
python3 -m peregrine_client.missions.trigger_surveillance --mode single --namespace uav4
```

The observed failure is:

```text
Waiting for action server execute_tree
Action server execute_tree not available
```

The same `execute_tree` path works in simulation.

Important distinction: TUI arm/takeoff/land does not use `execute_tree`. TUI talks directly to:

- `/uav4/arm`
- `/uav4/uav_manager/takeoff`
- `/uav4/uav_manager/land`

Surveillance uses:

- `/uav4/execute_tree`

So TUI working proves the UAV manager path is reachable, but it does not prove the behavior-tree action server exists or is bridged.

## Refined Diagnosis From Hardware Trace

## 2026-06-23 Follow-Up Finding

Live checks on UAV4 (`falinks@192.168.0.203`) showed the current startup and bridge
configuration are correct after a restart:

- `peregrine_tree_server` is running as `/uav4/bt_action_server`.
- `/uav4/execute_tree [btcpp_ros2_interfaces/action/ExecuteTree]` exists locally on
  the aircraft.
- `/tmp/zenoh_bridge.json5` exports `/uav4/execute_tree`.
- The GCS bridge imports `/uav.*/execute_tree` and currently discovers
  `/uav4/execute_tree`.

The failure observed in the logs is a server crash after two goal callbacks reached
the action server about 1 ms apart:

```text
[peregrine_tree_server-2] [INFO] ... Received goal request to execute Behavior Tree: MultiTrajectory
[peregrine_tree_server-2] [INFO] ... Received goal request to execute Behavior Tree: MultiTrajectory
[peregrine_tree_server-2] terminate called after throwing an instance of 'std::runtime_error'
[peregrine_tree_server-2]   what():  Failed to accept new goal
[ERROR] [peregrine_tree_server-2]: process has died ...
```

This does not prove that an operator sent two commands. The aircraft bridge reported
the requesting side as Zenoh bridge `19633adbf8e489a23fa855b0252dfcaf`; current
Docker logs for `ros2-px4-flight-gcs` do not contain that bridge ID, so the exact
origin is not recoverable from the retained logs. Plausible sources are a duplicated
Zenoh action request, a second/old GCS bridge, or another client process on the same
network. A follow-up source change logs the ROS action goal UUID so the next test can
separate same-UUID transport duplication from two distinct client goals.

Upstream context: this failure string comes from the ROS 2 action stack, not from
mission logic. ROS 2 actions use a goal UUID and action servers are expected to
handle simultaneous/racing goal requests; upstream `rclcpp` issue 3120 documents
a related crash where a duplicate goal UUID throws `std::runtime_error("Failed to
accept new goal")` instead of being rejected cleanly. This trace is therefore
consistent with a known ROS action-server failure mode, even if the exact
BehaviorTree.ROS2 `TreeExecutionServer` overlap path has not been confirmed as
the same upstream issue.

Once `peregrine_tree_server` dies, Zenoh undeclares `/uav4/execute_tree`; later GCS
commands then fail at `wait_for_server()` or hang on bridge requests with messages like:

```text
Route Service Client (ROS:/uav4/execute_tree/_action/send_goal -> ...):
received NO reply for request ... - cannot reply to client, it will hang until timeout
```

This is not explained by NTP skew in the checked state: the GCS host, GCS container,
UAV4 host, and UAV4 container clocks agreed within about one second, and both hosts
reported NTP synchronized. The `NO reply` bridge warning is consistent with the BT
server already being gone or unavailable to answer the action request.

Implemented local source fix:

- `src/behaviortree_ros2/behaviortree_ros2/src/tree_execution_server.cpp` now rejects
  overlapping `ExecuteTree` goals while one tree is active instead of allowing a second
  goal path to throw `Failed to accept new goal` and abort the process.
- The same source now logs the action goal UUID for each received/rejected goal.
- Verified with:

```bash
docker exec ros2-px4-flight-gcs bash -lc \
  'cd /ros2_ws && source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && \
   colcon build --packages-select behaviortree_ros2 peregrine_bt \
   --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'
```

Deploy/rebuild this patch on the aircraft before relying on repeated GCS mission
triggers. Do not restart the aircraft service while the vehicle is armed or flying.

Earlier tracing correctly narrowed the first split to whether `peregrine_tree_server`
is discoverable on UAV4. If `/uav4/execute_tree` is missing, it is either not
running or it started and exited/crashed before it remained discoverable.

The bridge is lower probability once these are true:

- GCS TUI arm/takeoff/land work from the same GCS tmux session.
- The rendered UAV and GCS Zenoh configs include the current action allowlists.
- The GCS container was restarted after config generation.

Those working commands prove the GCS can route actions to UAV4 through Zenoh. They do not prove `execute_tree`, because it is served by a different process:

| Command | Served by | Launched by |
|---|---|---|
| `/uav4/arm` | `uav_manager` | `core_stack.launch.py` |
| `/uav4/uav_manager/takeoff` | `uav_manager` | `core_stack.launch.py` |
| `/uav4/uav_manager/land` | `uav_manager` | `core_stack.launch.py` |
| `/uav4/execute_tree` | `peregrine_tree_server` / `bt_action_server` | `bt_mission.launch.py` |

So the most likely real split is:

- `core_stack.launch.py` is up, so TUI commands work.
- The BT server from `bt_mission.launch.py` is absent, crashed, or was never launched, so surveillance cannot start.

Common reasons:

- The aircraft was launched with only the core stack instead of `docker/config/start_flight_stack.sh` / `bt_mission.launch.py`.
- `peregrine_bt` or `btcpp_ros2_interfaces` is not built/installed in the UAV4 container overlay.
- `peregrine_tree_server` fails while loading/registering BT nodes or XML files from `peregrine_bt/trees`.
- A tree XML parse error kills the BT server during startup registration.
- A `groot2_port` conflict is possible, but from source it is normally hit when a tree goal is accepted and `BT::Groot2Publisher` is created, not during the initial `wait_for_server()` phase. Treat it as a log-check item, not the first explanation for "action server not available".

Source note: `behaviortree_ros2::TreeExecutionServer` creates the ROS action server in its constructor, then runs behavior-tree registration shortly after via a timer. If registration throws and the process exits, the client will still see `wait_for_server()` fail because the server disappeared before discovery completed.

## Most Likely Causes

Check in this order.

1. `peregrine_tree_server` is not running on the UAV4 companion computer.

   If the aircraft was started with `core_stack.launch.py` or an older startup script/image, the UAV manager actions can work while `/uav4/execute_tree` is missing. Current hardware startup should use `docker/config/start_flight_stack.sh`, which launches `peregrine_bringup bt_mission.launch.py`.

2. The UAV4 Zenoh bridge is using an old or wrong allowlist.

   The current UAV bridge must export:

   ```text
   /uav4/execute_tree
   ```

   under `action_servers`. If the aircraft image or checkout predates the `execute_tree` allowlist, takeoff/land can still work while surveillance cannot.

3. The GCS laptop was generated for the wrong hardware fleet.

   The local workstation currently has a sim `.env.local` for `uav1,uav2`; that is not valid for a hardware-only UAV4 GCS laptop. The GCS laptop needs hardware values such as:

   ```bash
   GCS_MODE=hardware
   GCS_UAV_IPS=<uav4-companion-ip>
   GCS_DRONE_IDS=4
   GCS_NUM_UAVS=1
   ```

   For multiple physical aircraft, keep IP order aligned with `GCS_DRONE_IDS`, for example:

   ```bash
   GCS_MODE=hardware
   GCS_UAV_IPS=<uav1-ip>,<uav4-ip>
   GCS_DRONE_IDS=1,4
   ```

4. The GCS command pane is in the wrong ROS environment.

   It must run in ROS domain 99, with the GCS Zenoh bridge running:

   ```bash
   export ROS_DOMAIN_ID=99
   export ROS_LOCALHOST_ONLY=0
   source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
   source /ros2_ws/install/setup.bash
   ```

5. The command is malformed.

   Use the module form and correct package spelling:

   ```bash
   python3 -m peregrine_client.missions.trigger_surveillance --mode single --namespace uav4
   ```

   Without `-m`, or with `peregrine_clinet`, Python should fail before ROS action discovery. If the log reaches "Waiting for action server execute_tree", the package was imported and this is not the primary issue.

   Also keep `--mode single` for UAV4-only hardware testing. `trigger_surveillance --mode multi` is hard-coded for `uav1` and `uav2`; it ignores `--namespace`.

## First Split: Is `/uav4/execute_tree` Local To The Aircraft?

Run these on the UAV4 companion computer.

```bash
cd /home/peregrine/peregrine/docker
cat .env.local
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
docker compose --env-file .env --env-file .env.local -f compose/docker-compose.jetson.yml config | egrep 'DRONE_ID|ZENOH|start_flight_stack|bt_mission|core_stack'
sudo systemctl status peregrine -l --no-pager || true
sudo journalctl -u peregrine -b -n 200 --no-pager || true
```

Enter the aircraft container. The container name is usually `ros2-px4-flight-aircraft-4` if `DRONE_ID=4`.

```bash
docker exec -it ros2-px4-flight-aircraft-4 bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=4
export ROS_LOCALHOST_ONLY=1

env | egrep 'DRONE_ID|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ZENOH'
ros2 node list | sort | egrep 'bt|tree|uav_manager|trajectory'
ros2 action list -t | sort | egrep 'execute_tree|takeoff|land'
ros2 action info /uav4/execute_tree
ps -ef | egrep 'peregrine_tree_server|bt_mission|core_stack' | grep -v grep
```

Expected good result:

```text
/uav4/execute_tree [btcpp_ros2_interfaces/action/ExecuteTree]
```

Interpretation:

- If `/uav4/uav_manager/takeoff` and `/uav4/uav_manager/land` exist but `/uav4/execute_tree` does not, the core stack is running but the BT server is not.
- If `/uav4/execute_tree` exists locally on the UAV but not on GCS, the problem is Zenoh bridge/export/import.

Temporary workaround if the core stack is already running but BT is missing:

```bash
ros2 launch peregrine_bringup bt_mission.launch.py \
  start_core_stack:=false \
  ros_domain_id:=4 \
  ros_localhost_only:=1 \
  uav_namespace:=/uav4 \
  use_sim_time:=false \
  groot2_port:=1673
```

If this makes `/uav4/execute_tree` appear, fix the aircraft startup path so it launches `bt_mission.launch.py`, not only `core_stack.launch.py`.

## Check The UAV4 Bridge Export

Still inside the UAV4 container:

```bash
sed -n '1,140p' /tmp/zenoh_bridge.json5
grep -n 'execute_tree\|takeoff\|land\|domain\|ros_localhost_only' /tmp/zenoh_bridge.json5
```

Expected good entries:

```text
domain: 4
ros_localhost_only: true
"/uav4/uav_manager/takeoff"
"/uav4/uav_manager/land"
"/uav4/execute_tree"
```

If takeoff/land are present but `execute_tree` is missing, the aircraft is running an old `docker/config/zenoh/uav_bridge.json5`. Pull the latest repo/submodules, rebuild/restart the aircraft image, or patch the bridge config and restart the aircraft container.

Relevant files:

- `docker/config/zenoh/uav_bridge.json5`
- `docker/config/start_flight_stack.sh`
- `docker/compose/docker-compose.jetson.yml`
- `docker/compose/docker-compose.rpi5.yml`

## Check The GCS Import

Run these on the GCS laptop.

```bash
cd /scratch/robotics/peregrine/docker
cat .env.local
docker compose --env-file .env --env-file .env.local -f compose/docker-compose.gcs.yml config | egrep 'GCS_|ZENOH|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY'
make gcs-down
make generate-gcs
sed -n '1,180p' config/zenoh/gcs_bridge.generated.json5
sed -n '1,120p' config/tmuxinator/gcs.generated.yml
make gcs
```

For UAV4 hardware, generated GCS config should connect to the UAV4 companion IP on port 7447:

```text
"tcp/<uav4-companion-ip>:7447"
```

The generated tmux config should show a UAV4 TUI pane if `GCS_DRONE_IDS=4` was set:

```text
uav4_tui: ros2 run tui_status tui_status_node --ros-args -p uav_namespace:=/uav4
```

Enter the GCS container:

```bash
docker exec -it ros2-px4-flight-gcs bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0

env | egrep 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|GCS_|ZENOH'
ps -ef | grep zenoh | grep -v grep
ros2 action list -t | sort | egrep 'uav4|execute_tree|takeoff|land'
ros2 action info /uav4/execute_tree
```

Expected good result in GCS domain 99:

```text
/uav4/execute_tree [btcpp_ros2_interfaces/action/ExecuteTree]
/uav4/uav_manager/takeoff [peregrine_interfaces/action/Takeoff]
/uav4/uav_manager/land [peregrine_interfaces/action/Land]
```

If takeoff/land show up but `execute_tree` does not:

- Re-check the UAV bridge allowlist for `/uav4/execute_tree`.
- Re-check the GCS bridge action client allowlist for `/uav.*/execute_tree`.
- If both are present, test an exact allowlist entry for `/uav4/execute_tree`; this would rule out a Zenoh bridge regex/version issue.

If no UAV4 actions show up:

- GCS is probably connected to the wrong IP/port or generated in `sim` mode.
- Check `docker/.env.local` and `docker logs ros2-px4-flight-gcs`.

## Direct Action Test

Only run this when the vehicle is physically safe to execute the mission.

From inside the GCS container:

```bash
ros2 action send_goal /uav4/execute_tree \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: SingleUavSurveillance, payload: ''}" \
  --feedback
```

If direct `ros2 action send_goal` works but the Python trigger does not, test the Python trigger with the exact same namespace:

```bash
python3 -m peregrine_client.missions.trigger_surveillance \
  --mode single \
  --namespace uav4
```

Alternative trigger using the newer trajectory trigger:

```bash
python3 -m peregrine_client.missions.trigger_trajectory \
  --uavs uav4 \
  --tree SingleUavSurveillance \
  --timeout 900
```

## Why Simulation Works

Simulation starts each UAV stack with `bt_mission.launch.py` in `docker/compose/docker-compose.multi-sitl.yml`, so each simulated UAV has:

```text
/uavN/execute_tree
```

The generated sim GCS config also connects to the generated sim bridge ports. Hardware can differ if:

- the aircraft service/image is older,
- the aircraft was launched with only `core_stack.launch.py`,
- the GCS laptop `.env.local` still targets sim or the wrong UAV IDs,
- the UAV/GCS Zenoh allowlists were not regenerated or rebuilt after `execute_tree` was added.

## Probable Fixes

Apply the fix matching the observed split.

If `/uav4/execute_tree` is missing on the aircraft:

1. Pull latest `peregrine` and submodules on the aircraft.
2. Rebuild the aircraft image.
3. Restart the flight service/container.
4. Confirm `docker/config/start_flight_stack.sh` launches `bt_mission.launch.py`.

If `/uav4/execute_tree` exists on the aircraft but not on GCS:

1. Confirm `/tmp/zenoh_bridge.json5` on UAV4 has `/uav4/execute_tree` in `action_servers`.
2. Confirm `docker/config/zenoh/gcs_bridge.generated.json5` on GCS has `/uav.*/execute_tree` in `action_clients`.
3. Restart both Zenoh bridges.
4. If still missing, replace regex allowlist entries with exact UAV4 action names as a diagnostic.

If GCS is configured for the wrong aircraft:

1. Set `docker/.env.local` on GCS to hardware mode with the UAV4 IP and `GCS_DRONE_IDS=4`.
2. Run `make gcs-down && make gcs`.
3. Confirm `/uav4/execute_tree` in `ros2 action list -t`.
