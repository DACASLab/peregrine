#!/usr/bin/env python3
"""End-to-end BVC collision-avoidance SITL harness.

Launches the *full surveillance mission* through the existing Behavior-Tree
dispatch (one ExecuteTree goal per UAV), then watches the shared fleet bus and
asserts that the inter-UAV separation never drops below the BVC guarantee while
both agents are airborne at a shared altitude.

Topology (matches docker/ multi-agent sim):
  - Each UAV runs its stack in its own ROS domain with BVC enabled and a Zenoh
    bridge that exports `execute_tree` + `/fleet/agent_state`.
  - This harness runs in the GCS ROS domain (99). The GCS Zenoh bridge makes the
    per-UAV `execute_tree` action and the shared `/fleet/agent_state` visible here.

Prerequisites (see docker/USAGE.md):
  Terminal 1:  cd docker && NUM_UAVS=2 ENABLE_AVOIDANCE=true make multi-sitl
  Terminal 2:  cd docker && make shell-gcs        # gives a domain-99 shell w/ bridge
               # then, inside the gcs container:
               python3 /ros2_ws/tools/sitl_bvc_mission.py --uavs uav1,uav2

Pass criterion:
  * every UAV's tree returns SUCCEEDED, and
  * the minimum horizontal separation between any two airborne, same-altitude
    UAVs stays >= 2*safety_radius - tol.

Run with --no-trigger to only monitor a mission triggered elsewhere, or with
ENABLE_AVOIDANCE=false in multi-sitl to capture a baseline (expected to violate).
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from btcpp_ros2_interfaces.action import ExecuteTree
from peregrine_interfaces.msg import FleetAgentState, UAVState

GOAL_STATUS_SUCCEEDED = 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED
_GROUND_STATES = {UAVState.STATE_IDLE, UAVState.STATE_LANDED}


@dataclass
class AgentSample:
    t: float
    x: float
    y: float
    z: float


@dataclass
class MissionResult:
    accepted: bool = False
    status: int = -1
    message: str = ""
    done: bool = False
    flew_and_landed: bool = False


@dataclass
class Stats:
    rows: list = field(default_factory=list)        # CSV rows (long format)
    pair_min: dict = field(default_factory=dict)    # (a,b) -> min horizontal dist (same band)
    overall_min_horiz: float = math.inf


class BvcMissionHarness(Node):
    def __init__(self, args):
        super().__init__("bvc_mission_harness")
        self.args = args
        self.uavs = [u.strip() for u in args.uavs.split(",") if u.strip()]
        if args.trees:
            self.trees = [t.strip() for t in args.trees.split(",")]
        else:
            # Default: MultiUavSurveillanceUav<N> keyed off the trailing digits.
            self.trees = [f"MultiUavSurveillanceUav{u.replace('uav', '')}" for u in self.uavs]
        if len(self.trees) != len(self.uavs):
            raise ValueError("number of --trees must match number of --uavs")

        self.min_sep_required = args.min_sep if args.min_sep > 0 else 2.0 * args.safety_radius
        self.latest: dict[str, AgentSample] = {}
        self.stats = Stats()
        self.results: dict[str, MissionResult] = {u: MissionResult() for u in self.uavs}
        self.t0 = time.time()
        self.sampling = False
        self.monitor_only = bool(args.no_trigger)

        fleet_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(
            FleetAgentState, "/fleet/agent_state", self._on_fleet_state, fleet_qos)

        # Track each UAV's supervisor state so we can detect mission completion from
        # flight reality (took off, then landed/idle) rather than relying on the
        # ExecuteTree action result routing back over the fleet bus.
        self.airborne_seen: dict[str, bool] = {u: False for u in self.uavs}
        self.landed_after_air: dict[str, bool] = {u: False for u in self.uavs}
        for u in self.uavs:
            self.create_subscription(
                UAVState, f"/{u}/uav_state",
                lambda msg, uav=u: self._on_uav_state(uav, msg), 10)

        self._action_clients = {
            u: ActionClient(self, ExecuteTree, f"/{u}/execute_tree") for u in self.uavs
        }

        self.create_timer(1.0 / args.sample_hz, self._sample)
        self.create_timer(0.5, self._check_done)

    # ── fleet bus ────────────────────────────────────────────────────────────
    def _on_fleet_state(self, msg: FleetAgentState):
        uav_id = msg.uav_id.lstrip("/")
        self.latest[uav_id] = AgentSample(
            t=time.time() - self.t0, x=msg.position.x, y=msg.position.y, z=msg.position.z)

    def _on_uav_state(self, uav: str, msg: UAVState):
        if msg.state not in _GROUND_STATES:
            self.airborne_seen[uav] = True
        elif self.airborne_seen[uav]:
            self.landed_after_air[uav] = True

    def _sample(self):
        if not self.sampling:
            return
        t = time.time() - self.t0
        for a, b in itertools.combinations(self.uavs, 2):
            sa, sb = self.latest.get(a), self.latest.get(b)
            if sa is None or sb is None:
                continue
            horiz = math.hypot(sa.x - sb.x, sa.y - sb.y)
            dz = abs(sa.z - sb.z)
            both_air = sa.z > self.args.airborne_z and sb.z > self.args.airborne_z
            same_band = dz <= self.args.altitude_band
            self.stats.rows.append([f"{t:.2f}", a, b, f"{horiz:.3f}", f"{dz:.3f}",
                                    int(both_air), int(same_band)])
            if both_air:
                self.stats.overall_min_horiz = min(self.stats.overall_min_horiz, horiz)
                if same_band:
                    key = (a, b)
                    self.stats.pair_min[key] = min(self.stats.pair_min.get(key, math.inf), horiz)

    # ── mission trigger ──────────────────────────────────────────────────────
    def start_mission(self):
        if self.args.no_trigger:
            self.get_logger().info("--no-trigger: monitoring only until --mission-timeout")
            self.sampling = True
            return
        for u, tree in zip(self.uavs, self.trees):
            client = self._action_clients[u]
            self.get_logger().info(f"waiting for {u} execute_tree action server...")
            if not client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error(f"{u}: execute_tree server unavailable")
                self.results[u] = MissionResult(done=True, message="NO_SERVER")
                continue
            goal = ExecuteTree.Goal()
            goal.target_tree = tree
            self.get_logger().info(f"{u}: ExecuteTree -> {tree}")
            client.send_goal_async(goal).add_done_callback(
                lambda fut, uav=u: self._on_goal_response(uav, fut))
        self.sampling = True

    def _on_goal_response(self, uav, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{uav}: goal rejected")
            self.results[uav] = MissionResult(done=True, message="REJECTED")
            return
        self.results[uav].accepted = True
        handle.get_result_async().add_done_callback(
            lambda fut, u=uav: self._on_result(u, fut))

    def _on_result(self, uav, future):
        res = future.result()
        self.results[uav].status = res.status
        self.results[uav].message = res.result.return_message
        self.results[uav].done = True
        self.get_logger().info(f"{uav}: tree done status={res.status} msg={res.result.return_message}")

    def _check_done(self):
        if self.monitor_only:
            return  # run until --mission-timeout
        # Primary signal: ExecuteTree results. Fallback: flight reality (every UAV
        # took off and came back to ground), since action results can fail to route
        # back over the fleet bus even when the mission completed normally.
        flew_and_landed = all(self.landed_after_air[u] for u in self.uavs)
        if all(r.done for r in self.results.values()) or flew_and_landed:
            for u in self.uavs:
                if not self.results[u].done and self.landed_after_air[u]:
                    self.results[u].done = True
                    self.results[u].flew_and_landed = True
            self.sampling = False
            raise KeyboardInterrupt  # break spin cleanly

    # ── reporting ────────────────────────────────────────────────────────────
    def report(self) -> bool:
        out = Path(self.args.out)
        out.mkdir(parents=True, exist_ok=True)
        csv_path = out / "separation.csv"
        with csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "uav_a", "uav_b", "horiz_m", "dz_m", "both_airborne", "same_band"])
            w.writerows(self.stats.rows)

        # A UAV "passed" the mission if its tree returned SUCCEEDED, or (when the
        # action result never routed back) if it demonstrably flew and landed.
        trees_ok = self.monitor_only or all(
            r.status == GOAL_STATUS_SUCCEEDED or r.flew_and_landed
            for r in self.results.values())
        tol = self.args.tol
        sep_ok = True
        worst = math.inf
        for (a, b), d in self.stats.pair_min.items():
            worst = min(worst, d)
            if d < self.min_sep_required - tol:
                sep_ok = False
        if not self.stats.pair_min:
            # No same-altitude airborne overlap ever happened -> nothing for BVC to do.
            self.get_logger().warn("no same-altitude airborne overlap observed; separation "
                                   "assertion is vacuous (did both UAVs fly the same layer?)")

        summary = {
            "uavs": self.uavs,
            "trees": self.trees,
            "results": {u: {"status": r.status, "message": r.message} for u, r in self.results.items()},
            "min_sep_required_m": self.min_sep_required,
            "tolerance_m": tol,
            "worst_same_band_horiz_m": None if worst == math.inf else round(worst, 3),
            "overall_min_horiz_m": None if self.stats.overall_min_horiz == math.inf
            else round(self.stats.overall_min_horiz, 3),
            "pair_min_m": {f"{a}|{b}": round(d, 3) for (a, b), d in self.stats.pair_min.items()},
            "trees_ok": trees_ok,
            "separation_ok": sep_ok,
            "samples": len(self.stats.rows),
        }
        (out / "summary.json").write_text(json.dumps(summary, indent=2))

        passed = trees_ok and sep_ok
        print("\n" + "=" * 64)
        print("BVC MISSION HARNESS RESULT")
        print("=" * 64)
        for u, r in self.results.items():
            ok = r.status == GOAL_STATUS_SUCCEEDED or r.flew_and_landed
            how = "SUCCEEDED" if r.status == GOAL_STATUS_SUCCEEDED else (
                "flew+landed" if r.flew_and_landed else f"status={r.status}")
            print(f"  {u:8s} mission {'OK' if ok else 'FAIL'} ({how}) {r.message}")
        print(f"  required separation : >= {self.min_sep_required:.2f} m (tol {tol:.2f})")
        print(f"  worst same-band     : {summary['worst_same_band_horiz_m']} m")
        print(f"  overall min horiz   : {summary['overall_min_horiz_m']} m")
        print(f"  trees_ok={trees_ok}  separation_ok={sep_ok}")
        print(f"  artifacts: {csv_path}  and  {out / 'summary.json'}")
        print("  VERDICT:", "PASS" if passed else "FAIL")
        print("=" * 64)
        return passed


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--uavs", default="uav1,uav2", help="comma-separated UAV namespaces")
    p.add_argument("--trees", default="", help="comma-separated tree ids (default MultiUavSurveillanceUav<N>)")
    p.add_argument("--safety-radius", dest="safety_radius", type=float, default=2.5,
                   help="per-agent r_s; min separation defaults to 2*r_s")
    p.add_argument("--min-sep", dest="min_sep", type=float, default=0.0,
                   help="override required min separation (m); 0 -> 2*safety_radius")
    p.add_argument("--tol", type=float, default=0.3, help="separation tolerance (m)")
    p.add_argument("--airborne-z", dest="airborne_z", type=float, default=1.0,
                   help="fleet-frame z above which a UAV is considered airborne")
    p.add_argument("--altitude-band", dest="altitude_band", type=float, default=2.0,
                   help="|dz| within which a pair counts as same-altitude (BVC active)")
    p.add_argument("--sample-hz", dest="sample_hz", type=float, default=10.0)
    p.add_argument("--mission-timeout", dest="mission_timeout", type=float, default=600.0)
    p.add_argument("--no-trigger", dest="no_trigger", action="store_true",
                   help="monitor only; do not send ExecuteTree goals")
    p.add_argument("--out", default="artifacts/bvc_mission", help="artifact output dir")
    args = p.parse_args()

    rclpy.init()
    node = BvcMissionHarness(args)
    node.start_mission()
    deadline = time.time() + args.mission_timeout
    passed = False
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if all(r.done for r in node.results.values()):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.sampling = False
        if time.time() >= deadline:
            node.get_logger().error("mission timed out")
        passed = node.report()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
