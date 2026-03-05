# FastRTPS Shared Memory Cross-Domain Data Leak in Multi-Container ROS 2 SITL

**Date:** March 2026
**ROS 2 Distro:** Humble Hawksbill
**Fast-DDS Version:** 2.6.x (Humble default)
**Platform:** Ubuntu 22.04, Linux 6.8.0, Docker with `network_mode: host`

---

## Summary

In a multi-UAV software-in-the-loop (SITL) setup using Docker containers with `network_mode: host`, we observed that ROS 2 subscriber nodes on DDS domain 99 received topic data published by nodes on DDS domains 1-9 — **without any configured bridge publishing on domain 99**. The data path bypassed the Zenoh DDS bridge entirely, flowing through FastRTPS shared memory (`/dev/shm`) which does not enforce DDS domain ID isolation.

This behavior is **not a feature** — it is an unintended side effect of how Fast-DDS implements its shared memory transport. It will not occur on real hardware where each device has its own physical memory space.

---

## Architecture

### Multi-UAV SITL Container Layout

```
┌─────────────────────────────────────────────────────────────────┐
│                        Host Machine                             │
│                     (network_mode: host)                        │
│                    shared /dev/shm across all                   │
│                                                                 │
│  ┌──────────────┐ ┌──────────────┐     ┌──────────────┐        │
│  │  UAV1         │ │  UAV2         │ ... │  UAV9         │        │
│  │  Domain ID: 1 │ │  Domain ID: 2 │     │  Domain ID: 9 │        │
│  │               │ │               │     │               │        │
│  │  Nodes:       │ │  Nodes:       │     │  Nodes:       │        │
│  │  - uav_manager│ │  - uav_manager│     │  - uav_manager│        │
│  │  - estimation │ │  - estimation │     │  - estimation │        │
│  │  - control    │ │  - control    │     │  - control    │        │
│  │  - trajectory │ │  - trajectory │     │  - trajectory │        │
│  │  - safety     │ │  - safety     │     │  - safety     │        │
│  │  - tf_bcast   │ │  - tf_bcast   │     │  - tf_bcast   │        │
│  │               │ │               │     │               │        │
│  │  Zenoh Bridge │ │  Zenoh Bridge │     │  Zenoh Bridge │        │
│  │  (CycloneDDS) │ │  (CycloneDDS) │     │  (CycloneDDS) │        │
│  │  port: 7447   │ │  port: 7448   │     │  port: 7455   │        │
│  └──────────────┘ └──────────────┘     └──────────────┘        │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  GCS Container                                           │    │
│  │  Domain ID: 99                                           │    │
│  │                                                           │    │
│  │  Nodes:                                                   │    │
│  │  - tui_status x9  (one per UAV, subscribes to /uavN/*)  │    │
│  │  - rviz2          (subscribes to /tf, /tf_static)        │    │
│  │  - viz_node x9    (subscribes to estimated_state, etc.)  │    │
│  │                                                           │    │
│  │  Zenoh Bridge (CycloneDDS)                               │    │
│  │  port: 7456, connects to 7447-7455                       │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

### Intended Data Flow

```
UAV container (domain N)          Zenoh network          GCS container (domain 99)
─────────────────────          ──────────────          ─────────────────────────
ROS node publishes              TCP transport           GCS Zenoh bridge subscribes
  on domain N                                             from Zenoh, publishes
        │                                                 to DDS domain 99
        ▼                                                       │
UAV Zenoh bridge ──────────────────────────────────────► GCS Zenoh bridge
  (CycloneDDS)        Zenoh peer-to-peer TCP              (CycloneDDS)
  subscribes from                                               │
  DDS domain N                                                  ▼
                                                        TUI / RViz / viz nodes
                                                          receive on domain 99
```

### Key Configuration Details

- **All containers**: `network_mode: host` (share host network namespace)
- **All containers**: Share host's `/dev/shm` (shared memory filesystem)
- **ROS nodes**: Use FastRTPS (`rmw_fastrtps_cpp`, Humble default). `RMW_IMPLEMENTATION` is NOT explicitly set.
- **Zenoh bridges**: Use CycloneDDS internally (via `cyclors` Rust crate). This is hardcoded in `zenoh-bridge-ros2dds` — NOT configurable.
- **`ROS_LOCALHOST_ONLY=1`**: Set in all containers. Restricts DDS UDP discovery to loopback interface.

---

## The Observed Behavior

### Setup

9-UAV SITL launched via `make multi-sitl`. GCS container running TUI status nodes (one per UAV) and RViz2, all on DDS domain 99.

### Condition: Loopback Multicast DISABLED

The Linux loopback interface (`lo`) does not have the `MULTICAST` flag enabled by default. Without running `sudo ip link set lo multicast on` on the **host machine**, the following behavior was observed:

#### Observation 1: TUI receives live data

The TUI status display for each UAV showed updating values — armed state, flight mode, position, altitude, GPS status, etc. The UAV could be commanded to arm and fly, and the TUI reflected state changes.

However, velocity values (vx, vy) during a circle trajectory did **not** oscillate as expected for world-frame velocities. This turned out to be correct behavior — the velocity is in body-frame FLU (Forward-Left-Up), where forward speed stays roughly constant during a coordinated circle.

#### Observation 2: RViz2 shows NO TF frames

RViz2 displayed "Global Status: Warn" with "frame [map] does not exist". No TF tree was visible.

#### Observation 3: Zero publishers on domain 99

```bash
# From GCS container tmux (domain 99):
$ ros2 topic info /uav1/uav_state -v
# Shows 0 publishers, 2 subscribers

$ ros2 topic info /tf -v
# Shows 0 publishers, 1 subscriber

$ ros2 topic list -v
# Published topics: ONLY viz/* topics (from viz nodes) and RViz tool topics
# NO peregrine_interfaces topics published
# NO /tf or /tf_static published
#
# Subscribed topics: ALL expected TUI/viz/RViz subscriptions present
```

Full `ros2 topic list -v` output (truncated):
```
Published topics:
 * /uav1/viz/actual_path [nav_msgs/msg/Path] 1 publisher
 * /uav1/viz/markers [visualization_msgs/msg/MarkerArray] 1 publisher
 * /uav1/viz/reference_path [nav_msgs/msg/Path] 1 publisher
 * ... (same for uav2-uav9)

Subscribed topics:
 * /tf [tf2_msgs/msg/TFMessage] 1 subscriber
 * /tf_static [tf2_msgs/msg/TFMessage] 1 subscriber
 * /uav1/uav_state [peregrine_interfaces/msg/UAVState] 2 subscribers
 * /uav1/estimated_state [peregrine_interfaces/msg/State] 2 subscribers
 * ... (all TUI/viz subscriptions for uav1-uav9)
```

#### Observation 4: CLI tools cannot receive data on domain 99

```bash
# Domain 99 — fails:
$ ros2 topic echo /uav1/uav_state --once --no-daemon
WARNING: topic [/uav1/uav_state] does not appear to be published yet
Could not determine the type for the passed topic

# Domain 1 — works:
$ ROS_DOMAIN_ID=1 ros2 topic echo /uav1/uav_state --once --no-daemon
header:
  stamp:
    sec: 498
    nanosec: 132000000
  frame_id: ''
state: 0
armed: false
...
```

#### Observation 5: ros2 topic hz shows no data on domain 99

```bash
$ ros2 topic hz /uav1/uav_state
# "topic not published yet" — indefinitely
```

### Condition: Loopback Multicast ENABLED

After running `sudo ip link set lo multicast on` on the **host** (not in any container):

- RViz2 immediately shows all TF frames
- `ros2 topic info /tf -v` shows the Zenoh bridge as publisher
- All topics visible via CLI tools
- TUI continues working (now through the proper Zenoh bridge path)

---

## Root Cause Analysis

### Why the Zenoh Bridge's DDS Side is Invisible

The Zenoh bridge (`zenoh-bridge-ros2dds`) uses **CycloneDDS** internally. The ROS 2 nodes use **FastRTPS**. These are different DDS implementations.

| Component | DDS Implementation | Shared Memory Mechanism |
|---|---|---|
| TUI, RViz, viz nodes | FastRTPS (Humble default) | Boost.interprocess |
| Zenoh bridge | CycloneDDS (hardcoded via `cyclors` crate) | iceoryx |

**CycloneDDS and FastRTPS cannot communicate via shared memory** — their shared memory implementations are completely incompatible (different libraries, different segment formats). They must discover each other through **UDP multicast** on the RTPS Simple Participant Discovery Protocol (SPDP).

With `ROS_LOCALHOST_ONLY=1`, DDS binds to `127.0.0.1`. UDP multicast discovery uses address `239.255.0.1`. On Linux, the loopback interface does not have the `MULTICAST` flag by default:

```bash
$ ip link show lo
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 ...
#    ^ no MULTICAST flag
```

Without the MULTICAST flag, SPDP multicast packets cannot be sent/received on loopback. The CycloneDDS participant (Zenoh bridge) and FastRTPS participants (ROS nodes) **cannot discover each other**. The bridge is running and has data from Zenoh, but its DDS publishers are invisible to all ROS nodes on domain 99.

### Why TUI Still Receives Data (The Leak)

FastRTPS has a built-in shared memory transport that is **enabled by default** alongside UDP. This transport uses files in `/dev/shm` for both discovery and data transfer between FastRTPS participants on the same host.

**Critical finding from the Fast-DDS 2.6.x source code** ([SharedMemTransport.cpp](https://github.com/eProsima/Fast-DDS/blob/2.6.x/src/cpp/rtps/transport/shared_mem/SharedMemTransport.cpp)):

```cpp
#define SHM_MANAGER_DOMAIN ("fastrtps")
```

The shared memory manager domain is a **hardcoded string**, not parameterized by DDS domain ID. Shared memory ports and segments are created without any domain scoping:

- **Port creation**: `open_port(port_id, ...)` — no domain parameter
- **Segment creation**: `create_segment(size, capacity)` — no domain parameter
- **Segment naming**: Uses a 16-character UUID — no domain embedded

**All FastRTPS participants on the same machine share the same shared memory space, regardless of their DDS domain ID.**

This means:
- UAV1's `uav_manager` (domain 1, FastRTPS) publishes `/uav1/uav_state` to shared memory
- TUI's subscriber (domain 99, FastRTPS) reads it from the **same shared memory space**
- The DDS domain boundary is not enforced at the shared memory transport layer
- Data flows directly: UAV node → `/dev/shm` → TUI node, **bypassing Zenoh entirely**

### Why CLI Tools Don't Get the Leak

`ros2 topic echo` and `ros2 topic hz` are short-lived processes. The shared memory endpoint matching appears to happen at participant startup time. By the time a CLI tool starts, the shared memory endpoints from other domains are already established and the late-joining CLI process cannot discover them through the shared memory discovery mechanism alone.

The long-running TUI nodes, started at container launch, are present during the initial shared memory discovery window and get matched with cross-domain publishers.

### Why /tf Doesn't Leak to RViz

Despite `/tf` being published by FastRTPS nodes in UAV containers (via `tf2_ros::TransformBroadcaster`), RViz does not receive it through the shared memory leak. We tested:

```bash
# Both fail on domain 99 without multicast (even with long-running processes):
$ ros2 topic echo /uav1/estimated_state --once --no-daemon  # FAILS
$ ros2 topic echo /tf --once --no-daemon                     # FAILS
```

Since both namespaced topics AND `/tf` fail for CLI tools, **the leak is exclusive to long-running nodes started during the shared memory discovery window**. The reason RViz doesn't get `/tf` is likely one or a combination of:

1. **Startup ordering**: RViz's `tf2_ros::TransformListener` creates a **dedicated `SingleThreadedExecutor` in a separate thread** with a `MutuallyExclusive` callback group. This secondary executor/participant context may initialize after the shared memory discovery window closes.

2. **Topic name collision**: `/tf` is published by **all 9 UAVs** on 9 different domains with the **same topic name**. The shared memory matching may fail or behave unpredictably with 9 publishers for one topic name across domains. In contrast, `/uav1/uav_state` has exactly one publisher globally — the topic name is unique.

3. **Participant GUID filtering**: The RTPS protocol layer above shared memory may still filter based on participant GUIDs that encode domain information, and RViz's tf2 listener may be stricter about this filtering than regular subscriptions.

The exact reason remains unconfirmed and would require deeper instrumentation of the FastRTPS shared memory discovery and matching logic.

---

## QoS Comparison (Ruling Out QoS Mismatch)

We verified that QoS is **not** the differentiating factor:

| Topic | Publisher QoS | Subscriber QoS | Compatible? |
|---|---|---|---|
| `/uavN/uav_state` | RELIABLE, VOLATILE, depth 10 | RELIABLE, VOLATILE, depth 10 | Yes |
| `/uavN/estimated_state` | RELIABLE, VOLATILE, depth 10 | RELIABLE, VOLATILE, depth 10 | Yes |
| `/tf` | RELIABLE, VOLATILE, depth 100 (`DynamicBroadcasterQoS`) | RELIABLE, VOLATILE, depth 100 (`DynamicListenerQoS`) | Yes |
| `/tf_static` | RELIABLE, TRANSIENT_LOCAL, depth 1 (`StaticBroadcasterQoS`) | RELIABLE, TRANSIENT_LOCAL, depth 100 (`StaticListenerQoS`) | Yes |

QoS profiles from [tf2_ros/qos.hpp](https://docs.ros2.org/galactic/api/tf2_ros/qos_8hpp_source.html):
- `DynamicBroadcasterQoS`: `rclcpp::QoS(100)` — reliable, volatile
- `StaticBroadcasterQoS`: `rclcpp::QoS(1)` + `transient_local()`
- `DynamicListenerQoS`: `rclcpp::QoS(100)` — reliable, volatile
- `StaticListenerQoS`: `rclcpp::QoS(100)` + `transient_local()`

---

## How to Reproduce

### Prerequisites

- Docker with `network_mode: host` for all containers
- Multiple ROS 2 Humble containers using default RMW (FastRTPS)
- Different `ROS_DOMAIN_ID` per container
- `ROS_LOCALHOST_ONLY=1` in all containers
- Loopback multicast **disabled**: `sudo ip link set lo multicast off`

### Steps

1. Launch multi-container setup where Container A (domain 1) publishes `/foo/bar` and Container B (domain 99) has a long-running node subscribing to `/foo/bar`.

2. **Do NOT** enable multicast on loopback. Do NOT run any DDS bridge.

3. Observe that the long-running subscriber in Container B receives data from Container A, despite being on a different DDS domain.

4. Verify with `ros2 topic list -v` on domain 99 — it will show **0 publishers** for `/foo/bar`, yet the subscriber node is receiving data.

5. Try `ros2 topic echo /foo/bar` on domain 99 — it will **fail** (short-lived process, misses the shared memory discovery window).

### Minimal Reproduction (Untested — Theoretical)

```bash
# Terminal 1: Publisher on domain 1
docker run --rm --network host -e ROS_DOMAIN_ID=1 -e ROS_LOCALHOST_ONLY=1 \
  ros:humble ros2 topic pub /test/msg std_msgs/msg/String "data: hello" --rate 10

# Terminal 2: Long-running subscriber on domain 99 (start quickly after terminal 1)
docker run --rm --network host -e ROS_DOMAIN_ID=99 -e ROS_LOCALHOST_ONLY=1 \
  ros:humble ros2 topic echo /test/msg

# Terminal 3: Check — should show 0 publishers on domain 99
docker run --rm --network host -e ROS_DOMAIN_ID=99 -e ROS_LOCALHOST_ONLY=1 \
  ros:humble ros2 topic info /test/msg -v
```

Note: The minimal reproduction may not trigger the leak if the timing of shared memory discovery doesn't align. The leak was reliably observed with `tmuxinator`-managed sessions where all nodes start near-simultaneously.

---

## Impact

### SITL (Current Setup)

- **TUI status displays work "by accident"** through the shared memory leak, not through the intended Zenoh bridge path
- **RViz TF does not work** without the multicast fix, making visualization debugging dependent on `ip link set lo multicast on`
- **Data accuracy is uncertain** — the leaked data may be stale, partial, or incorrectly matched if shared memory endpoints shift
- **The Zenoh bridge is being bypassed** for TUI data, meaning bridge configuration issues (QoS, topic allowlists) go undetected until deployment

### Hardware (Future)

- **This issue will NOT occur** on real hardware with physically separate machines, as each device has its own `/dev/shm`
- Any data flow issues masked by the shared memory leak in SITL will surface on hardware
- The Zenoh bridge path will become the only data path and must work correctly

---

## The Fix

The proper fix is enabling UDP multicast on the loopback interface so CycloneDDS (Zenoh bridge) and FastRTPS (ROS nodes) can discover each other through standard RTPS SPDP:

```bash
# Run on the HOST machine (not inside containers):
sudo ip link set lo multicast on
```

This does **not** persist across reboots. For persistence, create a systemd service:

```ini
# /etc/systemd/system/loopback-multicast.service
[Unit]
Description=Enable multicast on loopback for ROS 2 DDS discovery
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set lo multicast on
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now loopback-multicast.service
```

### Why Inside Containers Doesn't Work

Running `ip link set lo multicast on` inside a container with `network_mode: host` **does not work** because the container shares the host's network namespace. The `lo` interface is the host's loopback — modifying it requires host-level privileges that the container's capabilities may not include, or the change may not propagate correctly through the shared namespace.

---

## Key References

- [Fast-DDS SharedMemTransport.cpp (2.6.x)](https://github.com/eProsima/Fast-DDS/blob/2.6.x/src/cpp/rtps/transport/shared_mem/SharedMemTransport.cpp) — Source code showing `SHM_MANAGER_DOMAIN` hardcoded as `"fastrtps"`, no domain scoping on ports or segments
- [Fast-DDS Shared Memory Transport Documentation](https://fast-dds.docs.eprosima.com/en/v2.6.10/fastdds/transport/shared_memory/shared_memory.html) — Documentation is notably silent on domain isolation; segment IDs are random UUIDs with no domain component
- [Fast-DDS Transport Layer FAQ](https://fast-dds.docs.eprosima.com/en/3.x/fastdds/faq/transport_layer/transport_layer.html) — Confirms discovery traffic always uses UDP, even when shared memory is enabled for data
- [ROS 2 Domain ID Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) — Domain isolation is implemented through UDP port separation, not shared memory separation. Port overlap occurs after 120 participants per domain.
- [rmw_fastrtps Issue #676](https://github.com/ros2/rmw_fastrtps/issues/676) — Related: shared memory transport doesn't work with `initialPeersList` or discovery server, further indicating shared memory discovery is a separate, loosely-coupled mechanism
- [zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) — Zenoh bridge uses CycloneDDS (via `cyclors` crate), NOT FastRTPS. Incompatible shared memory implementations.
- [tf2_ros QoS definitions (qos.hpp)](https://docs.ros2.org/galactic/api/tf2_ros/qos_8hpp_source.html) — DynamicBroadcasterQoS, StaticBroadcasterQoS, listener equivalents

---

## Attempting a Minimal Reproducible Example

The leak was observed in a complex multi-container setup. To confirm and isolate the behavior, here are approaches for building a standalone reproduction — ordered from simplest to most thorough.

### Approach 1: Two Docker Containers, One Topic

The simplest possible test. Two `ros:humble` containers on different domains, same host network, shared `/dev/shm`.

**Pointers:**
- Disable loopback multicast first: `sudo ip link set lo multicast off`
- Container A: `network_mode: host`, `ROS_DOMAIN_ID=1`, `ROS_LOCALHOST_ONLY=1`. Run a long-running publisher node (Python or C++) on a uniquely-named topic like `/test/heartbeat` at ~10 Hz
- Container B: `network_mode: host`, `ROS_DOMAIN_ID=99`, `ROS_LOCALHOST_ONLY=1`. Run a long-running subscriber node on the same topic that logs to stdout when messages arrive
- **Key**: Both containers must start near-simultaneously (use `docker compose up`, not sequential `docker run`). The leak appears to depend on startup timing — participants that exist during the initial shared memory discovery window get matched
- Verify: `ros2 topic info /test/heartbeat -v` on domain 99 should show 0 publishers, yet the subscriber logs messages

**If it doesn't reproduce**: The timing window may be narrower than expected. Try launching both nodes from the same container with different `ROS_DOMAIN_ID` per node (e.g., via `ros2 run` with `--ros-args -r __domain_id:=...` or environment overrides in a launch file).

### Approach 2: Same Container, Two Domain IDs

Removes Docker networking from the equation entirely. Tests pure FastRTPS shared memory behavior.

**Pointers:**
- Single machine, no Docker
- Terminal 1: `ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1 ros2 run demo_nodes_cpp talker`
- Terminal 2: `ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 ros2 run demo_nodes_cpp listener`
- If the listener receives messages from the talker, the shared memory leak is confirmed at the most basic level
- Try with multicast both on and off to see if it changes the behavior

### Approach 3: Custom Node with Lifecycle Control

For controlled timing and to test the "discovery window" hypothesis.

**Pointers:**
- Write a minimal C++ node that creates a publisher, waits 5 seconds, then starts publishing. In a separate process (different domain), start a subscriber BEFORE the publisher begins. This tests whether the leak depends on both participants existing simultaneously at startup vs. one starting later
- Vary the delay between subscriber and publisher startup: 0s, 1s, 5s, 30s
- Log shared memory files: `ls /dev/shm/fastrtps*` before and after each launch to see what segments/ports exist and whether they change with domain ID

### Approach 4: Disable Shared Memory to Confirm Causation

The nuclear test — if disabling FastRTPS shared memory stops the leak, it confirms the root cause.

**Pointers:**
- Create a FastRTPS XML profile that disables shared memory:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_only</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_participant" is_default_profile="true">
      <rtps>
        <useBuiltinTransports>false</useBuiltinTransports>
        <userTransports>
          <transport_id>udp_only</transport_id>
        </userTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

- Set `FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/above.xml` in all containers
- Re-run Approach 1 or 2. If the cross-domain data stops flowing, shared memory was definitively the transport
- **Caveat**: This also disables shared memory for intra-domain communication, so legitimate same-domain performance will degrade. Only use for testing.

### Approach 5: Inspect `/dev/shm` Directly

Observational approach to understand the shared memory layout.

**Pointers:**
- With multi-container SITL running, inspect shared memory files:
  ```bash
  ls -la /dev/shm/fastrtps*
  ls -la /dev/shm/ | grep -i fast
  ```
- Note whether file names contain any domain-identifying information
- Count the number of port/segment files vs. number of participants across all domains
- Compare the files created with 1 UAV (domain 1 + domain 99) vs. 9 UAVs (domains 1-9 + domain 99)
- If all domains share the same port files, that directly confirms the lack of domain isolation

### What to Report

For any reproduction attempt, capture:
1. `ip link show lo` output (multicast flag status)
2. `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` for each process
3. `ros2 topic list -v` output on the subscriber's domain (showing 0 publishers)
4. Evidence that the subscriber received data (stdout logs, `ros2 topic hz`)
5. `ls -la /dev/shm/fastrtps*` output
6. `RMW_IMPLEMENTATION` value (or confirm it's unset / defaulting to FastRTPS)
7. Fast-DDS version: `dpkg -l | grep fastrtps` or `ros2 doctor --report | grep rmw`

---

## Open Questions

1. **Exact shared memory discovery mechanism**: How does FastRTPS shared memory discover endpoints? Is it through a global port file in `/dev/shm` that all participants on the host write to? If so, this would explain the cross-domain matching.

2. **Startup timing dependency**: Why do only long-running processes (started at container launch) get the leak, while CLI tools started later do not? Is there a discovery window that closes?

3. **Topic name multiplicity**: Does having multiple publishers with the same topic name (`/tf`) across domains cause the shared memory matching to fail, while unique topic names (`/uav1/uav_state`) succeed? Or is the `/tf` failure purely due to RViz's `TransformListener` architecture (dedicated executor thread)?

4. **Fast-DDS version differences**: Has eProsima addressed this in newer Fast-DDS versions (3.x)? The `SHM_MANAGER_DOMAIN` is still hardcoded in the latest source.

5. **Data integrity**: Is the leaked data guaranteed to be correct, or can shared memory cross-domain matching produce corrupted or mismatched data under race conditions?
