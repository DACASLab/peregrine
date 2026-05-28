#!/usr/bin/env python3
"""Headless SITL regression harness for Peregrine examples.

The harness has two execution modes:

* host mode: starts Docker containers, optionally rebuilds the workspace, then
  re-enters this script inside the simulation image.
* container mode: starts PX4/core/mission processes, monitors ROS topics, and
  writes compact artifacts.

The pass/fail signal is topic evidence, not log strings. Logs are still kept to
explain failures.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DOCKER_DIR = REPO_ROOT / "docker"
DEFAULT_ROS_DOMAIN_ID = 42
DEFAULT_ROS_LOCALHOST_ONLY = 1
DEFAULT_NUM_UAVS = 3


@dataclass(frozen=True)
class Expectation:
    armed_ever: bool = True
    offboard_ever: bool = True
    min_max_z: float = 4.5
    final_armed: bool = False
    final_z_below: float = 0.6
    require_bt_success: bool = False
    require_process_success: bool = False
    require_tree_load: bool = True


@dataclass(frozen=True)
class Case:
    name: str
    suite: str
    kind: str
    timeout_s: int
    expect: Expectation
    tree: str | None = None
    tree_id: str | None = None
    launch_package: str | None = None
    launch_file: str | None = None
    launch_args: dict[str, str] = field(default_factory=dict)
    description: str = ""


CASES: dict[str, Case] = {
    "bt/takeoff_hover_land": Case(
        name="bt/takeoff_hover_land",
        suite="bt",
        kind="bt",
        tree="takeoff_hover_land.xml",
        tree_id="TakeoffHoverLand",
        timeout_s=120,
        expect=Expectation(require_bt_success=True),
    ),
    "bt/multi_trajectory": Case(
        name="bt/multi_trajectory",
        suite="bt",
        kind="bt",
        tree="multi_trajectory.xml",
        tree_id="MultiTrajectory",
        timeout_s=220,
        expect=Expectation(require_bt_success=True),
    ),
    "bt/single_uav_surveillance": Case(
        name="bt/single_uav_surveillance",
        suite="bt",
        kind="bt",
        tree="single_uav_surveillance.xml",
        tree_id="SingleUavSurveillance",
        timeout_s=420,
        expect=Expectation(require_bt_success=True),
    ),
    "python/circle_figure8": Case(
        name="python/circle_figure8",
        suite="python-client",
        kind="launch",
        launch_package="hardware_abstraction_example",
        launch_file="circle_figure8_demo.launch.py",
        launch_args={"mission_type": "circle_figure8"},
        timeout_s=300,
        expect=Expectation(require_process_success=True),
    ),
    "python/controller_switch": Case(
        name="python/controller_switch",
        suite="python-client",
        kind="launch",
        launch_package="hardware_abstraction_example",
        launch_file="controller_switch_demo.launch.py",
        timeout_s=420,
        expect=Expectation(require_process_success=True),
    ),
    "python/multi_cycle": Case(
        name="python/multi_cycle",
        suite="python-client",
        kind="launch",
        launch_package="hardware_abstraction_example",
        launch_file="multi_cycle_demo.launch.py",
        launch_args={"multi_cycle_sequence": "circle,figure8"},
        timeout_s=420,
        expect=Expectation(require_process_success=True),
    ),
    "multi-uav/circle_figure8_3uav": Case(
        name="multi-uav/circle_figure8_3uav",
        suite="multi-uav",
        kind="multi_launch",
        launch_package="hardware_abstraction_example",
        launch_file="example14_multi_uav_circle_figure8.launch.py",
        launch_args={
            "num_uavs": str(DEFAULT_NUM_UAVS),
            "base_domain_id": "1",
            "mission_type": "circle_figure8",
            "uav_namespace_prefix": "uav",
            "inter_uav_start_delay_s": "1.0",
            "base_takeoff_altitude_m": "4.0",
            "takeoff_altitude_step_m": "1.0",
            "ros_localhost_only": "1",
        },
        timeout_s=420,
        expect=Expectation(require_process_success=True, min_max_z=3.5),
        description="Requires the multi-sitl compose stack to be up with 3 UAVs.",
    ),
}


def shlex_join(argv: list[str]) -> str:
    import shlex

    return " ".join(shlex.quote(str(item)) for item in argv)


def now_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def run(
    argv: list[str],
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    check: bool = True,
    capture: bool = False,
) -> subprocess.CompletedProcess[str]:
    print("+", shlex_join(argv), flush=True)
    return subprocess.run(
        argv,
        cwd=str(cwd) if cwd else None,
        env=env,
        text=True,
        capture_output=capture,
        check=check,
    )


def compose_args(compose_file: str) -> list[str]:
    args = ["docker", "compose", "--env-file", ".env"]
    if (DOCKER_DIR / ".env.local").exists():
        args += ["--env-file", ".env.local"]
    args += ["-f", compose_file]
    return args


def parse_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip("\"'")
        if key:
            values[key] = value
    return values


def project_name() -> str:
    values = parse_env_file(DOCKER_DIR / ".env")
    values.update(parse_env_file(DOCKER_DIR / ".env.local"))
    return values.get("PROJECT_NAME", "ros2-px4-flight")


def container_name(kind: str) -> str:
    base = project_name()
    if kind == "multi":
        return f"{base}-multi-sim"
    return f"{base}-sim"


def ros_bash(command: str) -> str:
    return (
        "set -e; "
        "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; "
        "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash; "
        f"{command}"
    )


def host_artifact_dir(base: Path | None, suite: str) -> Path:
    root = base or (REPO_ROOT / "artifacts" / "sitl")
    path = root / f"{now_stamp()}_{suite}"
    path.mkdir(parents=True, exist_ok=True)
    return path


def selected_cases(args: argparse.Namespace) -> list[Case]:
    if args.case:
        missing = [name for name in args.case if name not in CASES]
        if missing:
            raise SystemExit(f"Unknown case(s): {', '.join(missing)}")
        return [CASES[name] for name in args.case]
    if args.suite == "all":
        return list(CASES.values())
    if args.suite == "smoke":
        return [CASES["bt/takeoff_hover_land"], CASES["python/circle_figure8"]]
    return [case for case in CASES.values() if case.suite == args.suite]


def host_main(args: argparse.Namespace) -> int:
    cases = selected_cases(args)
    artifact_dir = host_artifact_dir(Path(args.artifact_dir) if args.artifact_dir else None, args.suite)
    summary_path = artifact_dir / "summary.json"

    if args.dry_run:
        print(json.dumps({"artifact_dir": str(artifact_dir), "cases": [case.name for case in cases]}, indent=2))
        return 0

    if args.build:
        build_cmd = ros_bash("cd /ros2_ws && colcon build --symlink-install")
        run(compose_args("compose/docker-compose.simulation.yml") + ["run", "--rm", "sim", "bash", "-lc", build_cmd], cwd=DOCKER_DIR)

    if args.suite == "multi-uav" or any(case.suite == "multi-uav" for case in cases):
        env = os.environ.copy()
        env["NUM_UAVS"] = str(args.num_uavs)
        env["HEADLESS"] = "1"
        run(["python3", "scripts/generate_multi_sitl.py", "--num-uavs", str(args.num_uavs)], cwd=DOCKER_DIR, env=env)
        run(compose_args("compose/docker-compose.multi-sitl.yml") + ["up", "-d"], cwd=DOCKER_DIR, env=env)
        try:
            return run_in_container(args, cases, artifact_dir, multi=True)
        finally:
            if args.keep_running:
                print("Leaving multi-sitl containers running.")
            else:
                run(compose_args("compose/docker-compose.multi-sitl.yml") + ["down"], cwd=DOCKER_DIR, check=False, env=env)

    run(compose_args("compose/docker-compose.simulation.yml") + ["up", "-d", "sim"], cwd=DOCKER_DIR)
    result = run_in_container(args, cases, artifact_dir, multi=False)
    if summary_path.exists():
        print(f"Summary: {summary_path}")
    return result


def run_in_container(args: argparse.Namespace, cases: list[Case], artifact_dir: Path, *, multi: bool) -> int:
    case_args: list[str] = []
    for case in cases:
        case_args += ["--case", case.name]
    cmd = [
        "python3",
        "/ros2_ws/tools/sitl_regression.py",
        "--in-container",
        "--artifact-dir",
        str(Path("/ros2_ws") / artifact_dir.relative_to(REPO_ROOT)),
        "--ros-domain-id",
        str(args.ros_domain_id),
        "--ros-localhost-only",
        str(args.ros_localhost_only),
        "--num-uavs",
        str(args.num_uavs),
    ] + case_args
    if multi:
        cmd.append("--multi-stack-running")
    if args.keep_running:
        cmd.append("--keep-running")
    return run(
        ["docker", "exec", container_name("multi" if multi else "sim"), "bash", "-lc", ros_bash(shlex_join(cmd))],
        cwd=DOCKER_DIR,
        check=False,
    ).returncode


def start_process(
    argv: list[str],
    log_path: Path,
    *,
    env: dict[str, str] | None = None,
    cwd: Path | None = None,
) -> subprocess.Popen[str]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("w", encoding="utf-8", errors="replace")
    print("+", shlex_join(argv), ">", log_path, flush=True)
    return subprocess.Popen(
        argv,
        cwd=str(cwd) if cwd else None,
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def terminate_process(proc: subprocess.Popen[str] | None, timeout_s: float = 3.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=timeout_s)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass


def cleanup_container_processes() -> None:
    patterns = [
        "ros2 launch peregrine_bringup",
        "ros2 launch hardware_abstraction_example",
        "/ros2_ws/install/peregrine_bt/lib/peregrine_bt/peregrine_tree_server",
        "component_container_mt",
        "MicroXRCEAgent",
        "make px4_sitl gz_x500",
        "/opt/PX4-Autopilot/build",
        "gz sim --verbose",
        "gz sim -v2",
    ]
    try:
        out = subprocess.check_output(["ps", "-eo", "pid=,cmd="], text=True)
    except Exception:
        return
    pids: list[int] = []
    for line in out.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, cmd = stripped.partition(" ")
        if any(pattern in cmd for pattern in patterns):
            try:
                pids.append(int(pid_text))
            except ValueError:
                pass
    if not pids:
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                pass
        time.sleep(2 if sig == signal.SIGTERM else 0)


def base_env(ros_domain_id: int, ros_localhost_only: int) -> dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "ROS_DOMAIN_ID": str(ros_domain_id),
            "ROS_LOCALHOST_ONLY": str(ros_localhost_only),
            "HEADLESS": "1",
            "PX4_PARAM_UXRCE_DDS_PTCFG": "1",
            "PX4_PARAM_UXRCE_DDS_DOM_ID": str(ros_domain_id),
            "PX4_PARAM_UXRCE_DDS_SYNCT": "0",
        }
    )
    return env


def wait_for_node(node_name: str, env: dict[str, str], timeout_s: float = 15.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        proc = subprocess.run(["ros2", "node", "list"], env=env, text=True, capture_output=True)
        if proc.returncode == 0 and node_name in proc.stdout.splitlines():
            return True
        time.sleep(0.5)
    return False


def lifecycle_set(node_name: str, transition: str, env: dict[str, str], timeout_s: int = 15) -> int:
    proc = subprocess.run(
        ["timeout", str(timeout_s), "ros2", "lifecycle", "set", node_name, transition],
        env=env,
        text=True,
        capture_output=True,
    )
    return proc.returncode


def monitor_topics(duration_s: float, namespace: str = "", px4_namespace: str = "") -> dict[str, Any]:
    import rclpy
    from peregrine_interfaces.msg import GpsStatus, PX4Status, SafetyStatus, State, UAVState
    from rclpy.executors import SingleThreadedExecutor
    from px4_msgs.msg import VehicleStatus
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    def topic(name: str) -> str:
        ns = namespace.strip("/")
        return f"/{ns}/{name.lstrip('/')}" if ns else f"/{name.lstrip('/')}"

    def px4_topic(name: str) -> str:
        ns = px4_namespace.strip("/")
        return f"/{ns}/{name.lstrip('/')}" if ns else f"/{name.lstrip('/')}"

    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node("sitl_regression_monitor", context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    obs: dict[str, Any] = {
        "state_samples": 0,
        "first_z": None,
        "last_z": None,
        "min_z": None,
        "max_z": None,
        "uav_samples": 0,
        "uav_states": [],
        "armed_ever": False,
        "offboard_ever": False,
        "dependencies_ready_ever": False,
        "last_uav": None,
        "vehicle_status_samples": 0,
        "nav_states": [],
        "arming_states": [],
        "preflight_pass_ever": False,
        "last_vehicle_status": None,
        "safety_samples": 0,
        "safety_levels": [],
        "last_safety": None,
        "px4_status_samples": 0,
        "last_px4_status": None,
        "gps_samples": 0,
        "last_gps": None,
    }

    def state_cb(msg: Any) -> None:
        z = float(msg.pose.pose.position.z)
        obs["state_samples"] += 1
        if obs["first_z"] is None:
            obs["first_z"] = z
        obs["last_z"] = z
        obs["min_z"] = z if obs["min_z"] is None else min(obs["min_z"], z)
        obs["max_z"] = z if obs["max_z"] is None else max(obs["max_z"], z)

    def uav_cb(msg: Any) -> None:
        state = int(msg.state)
        obs["uav_samples"] += 1
        if state not in obs["uav_states"]:
            obs["uav_states"].append(state)
        obs["armed_ever"] = bool(obs["armed_ever"] or msg.armed)
        obs["offboard_ever"] = bool(obs["offboard_ever"] or msg.offboard)
        obs["dependencies_ready_ever"] = bool(obs["dependencies_ready_ever"] or msg.dependencies_ready)
        obs["last_uav"] = {
            "state": state,
            "armed": bool(msg.armed),
            "offboard": bool(msg.offboard),
            "connected": bool(msg.connected),
            "failsafe": bool(msg.failsafe),
            "mode": msg.mode,
            "dependencies_ready": bool(msg.dependencies_ready),
            "detail": msg.detail,
            "readiness_detail": msg.readiness_detail,
        }

    def vehicle_status_cb(msg: Any) -> None:
        nav = int(msg.nav_state)
        arm = int(msg.arming_state)
        obs["vehicle_status_samples"] += 1
        if nav not in obs["nav_states"]:
            obs["nav_states"].append(nav)
        if arm not in obs["arming_states"]:
            obs["arming_states"].append(arm)
        obs["preflight_pass_ever"] = bool(obs["preflight_pass_ever"] or msg.pre_flight_checks_pass)
        obs["last_vehicle_status"] = {
            "nav_state": nav,
            "arming_state": arm,
            "pre_flight_checks_pass": bool(msg.pre_flight_checks_pass),
            "failsafe": bool(msg.failsafe),
        }

    def safety_cb(msg: Any) -> None:
        level = int(msg.level)
        obs["safety_samples"] += 1
        if level not in obs["safety_levels"]:
            obs["safety_levels"].append(level)
        obs["last_safety"] = {
            "level": level,
            "nominal": level == SafetyStatus.LEVEL_NOMINAL,
            "reason": msg.reason,
        }

    def px4_cb(msg: Any) -> None:
        obs["px4_status_samples"] += 1
        obs["last_px4_status"] = {
            "connected": bool(msg.connected),
            "armed": bool(msg.armed),
            "offboard": bool(msg.offboard),
            "failsafe": bool(msg.failsafe),
            "nav_state": int(msg.nav_state),
            "arming_state": int(msg.arming_state),
            "battery_remaining": float(msg.battery_remaining),
        }

    def gps_cb(msg: Any) -> None:
        obs["gps_samples"] += 1
        obs["last_gps"] = {
            "fix_type": int(msg.fix_type),
            "hdop": float(msg.hdop),
            "vdop": float(msg.vdop),
            "satellites_used": int(msg.satellites_used),
        }

    node.create_subscription(State, topic("state"), state_cb, 10)
    node.create_subscription(State, topic("estimated_state"), state_cb, 10)
    node.create_subscription(UAVState, topic("uav_state"), uav_cb, 10)
    node.create_subscription(VehicleStatus, px4_topic("fmu/out/vehicle_status_v1"), vehicle_status_cb, sensor_qos)
    node.create_subscription(SafetyStatus, topic("safety_status"), safety_cb, 10)
    node.create_subscription(PX4Status, topic("status"), px4_cb, 10)
    node.create_subscription(GpsStatus, topic("gps_status"), gps_cb, 10)

    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline and ctx.ok():
        executor.spin_once(timeout_sec=0.2)
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown(context=ctx)
    return obs


def read_log(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def evaluate(
    case: Case,
    observation: dict[str, Any],
    logs: dict[str, Path],
    process_rc: int | None,
    configure_rc: int | None = None,
    activate_rc: int | None = None,
) -> tuple[str, list[str]]:
    failures: list[str] = []
    expect = case.expect

    if case.kind == "bt":
        if configure_rc is not None and configure_rc != 0:
            failures.append(f"BT configure failed (rc={configure_rc})")
        if activate_rc is not None and activate_rc != 0:
            failures.append(f"BT activate failed (rc={activate_rc})")

    max_z = observation.get("max_z")
    last_z = observation.get("last_z")
    last_uav = observation.get("last_uav") or {}

    if expect.require_tree_load and "bt" in logs:
        bt_text = read_log(logs["bt"])
        loaded_marker = f"Loaded BehaviorTree: {case.tree}" if case.tree else ""
        created_marker = f"Tree '{case.tree_id}' created" if case.tree_id else ""
        requested_tree_loaded = bool(
            (loaded_marker and loaded_marker in bt_text) or
            (created_marker and created_marker in bt_text)
        )
        if not requested_tree_loaded:
            failures.append(f"BT tree {case.tree or case.tree_id} did not load")
        if "Node not recognized" in bt_text:
            failures.append("BT node type not recognized")
    if expect.require_bt_success:
        bt_text = read_log(logs["bt"]) if "bt" in logs else ""
        goal_text = read_log(logs["goal"]) if "goal" in logs else ""
        if ("Tree finished with status: SUCCESS" not in bt_text
                and "Tree completed with SUCCESS" not in bt_text
                and "SUCCEEDED" not in goal_text):
            failures.append("BT did not complete with SUCCESS")
    if expect.require_process_success and process_rc not in (0, None):
        failures.append(f"mission process exited with rc={process_rc}")
    if expect.armed_ever and not observation.get("armed_ever"):
        failures.append("never armed")
    if expect.offboard_ever and not observation.get("offboard_ever"):
        failures.append("never entered offboard")
    if max_z is None or float(max_z) < expect.min_max_z:
        failures.append(f"max altitude {max_z} < {expect.min_max_z}")
    if not expect.final_armed and bool(last_uav.get("armed")):
        failures.append("final UAV state is still armed")
    if last_z is None or float(last_z) > expect.final_z_below:
        failures.append(f"final altitude {last_z} > {expect.final_z_below}")
    return ("pass" if not failures else "fail"), failures


def summarize_bt_log(path: Path) -> dict[str, Any]:
    text = read_log(path)
    return {
        "loaded": "Loaded BehaviorTree" in text or "Tree '" in text,
        "success": "Tree finished with status: SUCCESS" in text or "Tree completed with SUCCESS" in text,
        "failure": "Tree finished with status: FAILURE" in text or "Tree completed with FAILURE" in text,
        "failed_to_load": "Failed to load" in text,
        "send_goal_timeout": "SEND_GOAL_TIMEOUT" in text,
        "server_unreachable": "SERVER_UNREACHABLE" in text or "not reachable" in text,
        "node_not_recognized": "Node not recognized" in text,
    }


def run_single_case(case: Case, args: argparse.Namespace, artifact_dir: Path) -> dict[str, Any]:
    case_dir = artifact_dir / case.name.replace("/", "__")
    case_dir.mkdir(parents=True, exist_ok=True)
    env = base_env(args.ros_domain_id, args.ros_localhost_only)
    procs: list[subprocess.Popen[str]] = []
    logs: dict[str, Path] = {}
    mission_rc: int | None = None

    cleanup_container_processes()
    try:
        px4_log = case_dir / "px4.log"
        logs["px4"] = px4_log
        procs.append(
            start_process(
                ["bash", "-lc", "cd /opt/PX4-Autopilot && make px4_sitl gz_x500"],
                px4_log,
                env=env,
            )
        )

        if case.kind == "bt":
            core_log = case_dir / "core.log"
            bt_log = case_dir / "bt.log"
            logs["core"] = core_log
            logs["bt"] = bt_log
            procs.append(
                start_process(
                    [
                        "ros2",
                        "launch",
                        "peregrine_bringup",
                        "core_stack.launch.py",
                        f"ros_domain_id:={args.ros_domain_id}",
                        f"ros_localhost_only:={args.ros_localhost_only}",
                        "use_sim_time:=true",
                        "start_microxrce_agent:=true",
                        "start_visualizer:=false",
                        "start_rviz:=false",
                    ],
                    core_log,
                    env=env,
                )
            )
            procs.append(
                start_process(
                    [
                        "ros2",
                        "launch",
                        "peregrine_bringup",
                        "bt_mission.launch.py",
                        "start_core_stack:=false",
                        "use_sim_time:=true",
                    ],
                    bt_log,
                    env=env,
                )
            )
            wait_for_node("/bt_action_server", env, timeout_s=30.0)
            goal_log = case_dir / "goal.log"
            logs["goal"] = goal_log
            procs.append(
                start_process(
                    [
                        "ros2",
                        "action",
                        "send_goal",
                        "/execute_tree",
                        "btcpp_ros2_interfaces/action/ExecuteTree",
                        f"{{target_tree: '{case.tree_id}', payload: ''}}",
                        "--feedback",
                    ],
                    goal_log,
                    env=env,
                )
            )
            configure_rc = None
            activate_rc = None
            bt_lifecycle_ok = True
        elif case.kind == "launch":
            mission_log = case_dir / "mission.log"
            logs["mission"] = mission_log
            launch_args = [
                "ros2",
                "launch",
                str(case.launch_package),
                str(case.launch_file),
                f"ros_domain_id:={args.ros_domain_id}",
                f"ros_localhost_only:={args.ros_localhost_only}",
            ]
            for key, value in case.launch_args.items():
                launch_args.append(f"{key}:={value}")
            proc = start_process(launch_args, mission_log, env=env)
            procs.append(proc)
            configure_rc = None
            activate_rc = None
            bt_lifecycle_ok = True
        else:
            raise RuntimeError(f"Unsupported single case kind: {case.kind}")

        monitor_duration = case.timeout_s if bt_lifecycle_ok else 10.0
        observation = monitor_topics(monitor_duration)
        if case.kind == "launch":
            mission_rc = procs[-1].poll()
        if bool((observation.get("last_uav") or {}).get("armed")) or (observation.get("last_z") or 0.0) > 0.8:
            land_log = case_dir / "manual_land.log"
            logs["manual_land"] = land_log
            start_process(
                [
                    "timeout",
                    "60s",
                    "ros2",
                    "action",
                    "send_goal",
                    "/uav_manager/land",
                    "peregrine_interfaces/action/Land",
                    "{descent_velocity_mps: 0.8}",
                    "--feedback",
                ],
                land_log,
                env=env,
            ).wait(timeout=70)
            observation["post_land"] = monitor_topics(20)
        status, failures = evaluate(case, observation, logs, mission_rc, configure_rc, activate_rc)
        result = {
            "name": case.name,
            "suite": case.suite,
            "status": status,
            "failures": failures,
            "observation": observation,
            "logs": {key: str(value) for key, value in logs.items()},
            "process_rc": mission_rc,
            "configure_rc": configure_rc,
            "activate_rc": activate_rc,
        }
        if "bt" in logs:
            result["bt_log_summary"] = summarize_bt_log(logs["bt"])
        (case_dir / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
        return result
    finally:
        if not args.keep_running:
            for proc in reversed(procs):
                terminate_process(proc)
            cleanup_container_processes()


def monitor_domain_subprocess(
    domain: int,
    namespace: str,
    duration_s: int,
    out_path: Path,
    px4_namespace: str = "",
) -> subprocess.Popen[str]:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    return subprocess.Popen(
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "--monitor-only",
            "--duration-s",
            str(duration_s),
            "--namespace",
            namespace,
            "--px4-namespace",
            px4_namespace,
            "--output",
            str(out_path),
        ],
        env=env,
        text=True,
    )


def run_multi_case(case: Case, args: argparse.Namespace, artifact_dir: Path) -> dict[str, Any]:
    case_dir = artifact_dir / case.name.replace("/", "__")
    case_dir.mkdir(parents=True, exist_ok=True)
    mission_log = case_dir / "mission.log"
    logs = {"mission": mission_log}

    env = os.environ.copy()
    env["ROS_LOCALHOST_ONLY"] = str(args.ros_localhost_only)
    env["NUM_UAVS"] = str(args.num_uavs)
    launch_args = [
        "ros2",
        "launch",
        str(case.launch_package),
        str(case.launch_file),
    ]
    merged_args = dict(case.launch_args)
    merged_args["num_uavs"] = str(args.num_uavs)
    for key, value in merged_args.items():
        launch_args.append(f"{key}:={value}")

    monitors: list[tuple[int, str, Path, subprocess.Popen[str]]] = []
    for i in range(args.num_uavs):
        domain = i + 1
        namespace = f"/uav{domain}"
        px4_namespace = "" if i == 0 else f"/px4_{i}"
        out = case_dir / f"monitor_uav{domain}.json"
        monitors.append(
            (
                domain,
                namespace,
                out,
                monitor_domain_subprocess(domain, namespace, case.timeout_s, out, px4_namespace),
            )
        )

    proc = start_process(launch_args, mission_log, env=env)
    try:
        try:
            proc.wait(timeout=case.timeout_s + 60)
        except subprocess.TimeoutExpired:
            pass
        observations: dict[str, Any] = {}
        for domain, namespace, out, monitor_proc in monitors:
            try:
                monitor_proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                monitor_proc.terminate()
            if out.exists():
                observations[f"uav{domain}"] = json.loads(out.read_text(encoding="utf-8"))
            else:
                observations[f"uav{domain}"] = {"error": "missing monitor output", "namespace": namespace}
        failures: list[str] = []
        per_uav_status: dict[str, str] = {}
        for uav_name, obs in observations.items():
            status, obs_failures = evaluate(case, obs, logs, proc.poll())
            per_uav_status[uav_name] = status
            failures += [f"{uav_name}: {failure}" for failure in obs_failures]
        if case.expect.require_process_success and proc.poll() not in (0, None):
            failures.append(f"mission process exited with rc={proc.poll()}")
        result = {
            "name": case.name,
            "suite": case.suite,
            "status": "pass" if not failures else "fail",
            "failures": failures,
            "per_uav_status": per_uav_status,
            "observations": observations,
            "logs": {key: str(value) for key, value in logs.items()},
            "process_rc": proc.poll(),
        }
        (case_dir / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
        return result
    finally:
        if not args.keep_running:
            terminate_process(proc)


def write_summary(results: list[dict[str, Any]], artifact_dir: Path) -> int:
    passed = sum(1 for result in results if result["status"] == "pass")
    failed = len(results) - passed
    summary = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "passed": passed,
        "failed": failed,
        "results": results,
    }
    (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

    lines = [f"# SITL Regression Summary", "", f"Passed: {passed}", f"Failed: {failed}", ""]
    for result in results:
        lines.append(f"## {result['name']}: {result['status'].upper()}")
        failures = result.get("failures") or []
        if failures:
            for failure in failures:
                lines.append(f"- {failure}")
        obs = result.get("observation")
        if obs:
            lines.append(
                f"- max_z={obs.get('max_z')} last_z={obs.get('last_z')} "
                f"armed_ever={obs.get('armed_ever')} offboard_ever={obs.get('offboard_ever')}"
            )
        lines.append("")
    (artifact_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print(f"Artifacts: {artifact_dir}")
    return 0 if failed == 0 else 1


def container_main(args: argparse.Namespace) -> int:
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = str(args.ros_localhost_only)
    artifact_dir = Path(args.artifact_dir)
    artifact_dir.mkdir(parents=True, exist_ok=True)
    cases = selected_cases(args)
    results: list[dict[str, Any]] = []
    for case in cases:
        print(f"=== {case.name} ===", flush=True)
        if case.kind == "multi_launch":
            results.append(run_multi_case(case, args, artifact_dir))
        else:
            results.append(run_single_case(case, args, artifact_dir))
    return write_summary(results, artifact_dir)


def monitor_only_main(args: argparse.Namespace) -> int:
    observation = monitor_topics(args.duration_s, args.namespace, args.px4_namespace)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(observation, indent=2, sort_keys=True), encoding="utf-8")
    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=["bt", "python-client", "multi-uav", "smoke", "all"], default="smoke")
    parser.add_argument("--case", action="append", help="Run one case by name. Can be repeated.")
    parser.add_argument("--artifact-dir", default=None)
    parser.add_argument("--build", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--keep-running", action="store_true")
    parser.add_argument("--ros-domain-id", type=int, default=DEFAULT_ROS_DOMAIN_ID)
    parser.add_argument("--ros-localhost-only", type=int, default=DEFAULT_ROS_LOCALHOST_ONLY)
    parser.add_argument("--num-uavs", type=int, default=DEFAULT_NUM_UAVS)
    parser.add_argument("--in-container", action="store_true")
    parser.add_argument("--multi-stack-running", action="store_true")
    parser.add_argument("--monitor-only", action="store_true")
    parser.add_argument("--duration-s", type=int, default=60)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--px4-namespace", default="")
    parser.add_argument("--output", default="/tmp/sitl_monitor.json")
    parser.add_argument("--list", action="store_true", help="List available cases.")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.list:
        for name, case in CASES.items():
            print(f"{name}\t{case.suite}\t{case.kind}\t{case.timeout_s}s")
        return 0
    if args.monitor_only:
        return monitor_only_main(args)
    if args.in_container:
        return container_main(args)
    return host_main(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
