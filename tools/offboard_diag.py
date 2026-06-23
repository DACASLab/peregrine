#!/usr/bin/env python3
"""Offboard-timeout diagnostic for the Peregrine PX4 stack.

WHY THIS EXISTS
---------------
uav_manager commands offboard by publishing VEHICLE_CMD_SET_NAV_STATE and then
polling for vehicle_status.nav_state == OFFBOARD. If PX4 refuses the switch, the
stack only reports the opaque "OFFBOARD_TIMEOUT" -- it never says why.

PX4 refuses / drops OFFBOARD for two dominant reasons, both observable from the
topics this aircraft already bridges over uXRCE-DDS:

  1. The pilot took over on RC. vehicle_status.failsafe_and_user_took_over goes
     true and nav_state_user_intention diverges from nav_state; PX4 will not hand
     control back to OFFBOARD until that clears (RC mode switch returned / sticks
     centered).
  2. The local position/velocity estimate is invalid. OFFBOARD with position or
     velocity setpoints requires vehicle_local_position.{xy_valid,z_valid,
     v_xy_valid,v_z_valid}; if any required flag is false (or dead_reckoning is
     true) PX4 rejects/exits OFFBOARD. This is the "GPS / Z-altitude errors threw
     things off" failure mode.

A third, rarer cause is a stale offboard heartbeat (<2 Hz on
/fmu/in/offboard_control_mode), which this also measures.

Run it alongside a takeoff/mission attempt. When OFFBOARD entry fails it prints,
at that instant, exactly which precondition was not met.

NOTE: failsafe_flags and vehicle_command_ack would give the most explicit answer,
but PX4's dds_topics.yaml on this airframe does not bridge them. See the companion
notes for how to add them for an even more direct readout.

USAGE (on the aircraft companion, ROS domain 4):
  docker exec -it ros2-px4-flight-aircraft-4 bash -lc '
    source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=4 ROS_LOCALHOST_ONLY=1
    python3 /ros2_ws/tools/offboard_diag.py'
"""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, OffboardControlMode


NAV_STATE = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER",
    5: "AUTO_RTL", 6: "POSITION_SLOW", 10: "ACRO", 12: "DESCEND", 13: "TERMINATION",
    14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
    19: "AUTO_FOLLOW", 20: "AUTO_PRECLAND", 21: "ORBIT", 22: "AUTO_VTOL_TAKEOFF",
}
ARMING_STATE = {1: "DISARMED", 2: "ARMED"}
NAV_STATE_OFFBOARD = 14

# vehicle_local_position validity flags that OFFBOARD depends on, plus context.
POS_FLAGS = ["xy_valid", "z_valid", "v_xy_valid", "v_z_valid",
             "heading_good_for_control", "xy_global", "z_global", "dead_reckoning"]


def px4_qos() -> QoSProfile:
    """PX4 publishes uORB topics BEST_EFFORT / KEEP_LAST; match it or get nothing."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class OffboardDiag(Node):
    def __init__(self, px4_ns: str):
        super().__init__("offboard_diag")
        self.px4_ns = "" if px4_ns in ("", "/") else ("/" + px4_ns.strip("/"))

        self.status: VehicleStatus | None = None
        self.lpos: VehicleLocalPosition | None = None
        self.last_nav = None
        self.last_took_over = None
        self.hb_times: list[float] = []

        topics = self._resolve_topics()
        qos = px4_qos()
        self.create_subscription(VehicleStatus, topics["vehicle_status"], self._on_status, qos)
        if topics.get("vehicle_local_position"):
            self.create_subscription(
                VehicleLocalPosition, topics["vehicle_local_position"], self._on_lpos, qos)
        else:
            self.get_logger().warn("vehicle_local_position not found -- position blockers unavailable")
        if topics.get("offboard_control_mode"):
            self.create_subscription(
                OffboardControlMode, topics["offboard_control_mode"], self._on_heartbeat, qos)

        self.create_timer(1.0, self._tick)
        self.get_logger().info("offboard_diag running. Trigger your takeoff/mission now.")

    def _resolve_topics(self) -> dict[str, str]:
        """Match topics by base name, ignoring PX4 _vN version suffixes."""
        wanted = {
            "vehicle_status": "fmu/out/vehicle_status",
            "vehicle_local_position": "fmu/out/vehicle_local_position",
            "offboard_control_mode": "fmu/in/offboard_control_mode",
        }
        found: dict[str, str] = {}
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(found) < len(wanted):
            for name, _types in self.get_topic_names_and_types():
                for key, base in wanted.items():
                    if key in found:
                        continue
                    target = (self.px4_ns + "/" + base) if self.px4_ns else ("/" + base)
                    if name == target or name.startswith(target + "_v"):
                        found[key] = name
            if len(found) < len(wanted):
                rclpy.spin_once(self, timeout_sec=0.3)
        self.get_logger().info("Resolved topics: " + ", ".join(f"{k}={v}" for k, v in found.items()))
        if "vehicle_status" not in found:
            self.get_logger().error("vehicle_status topic not found -- is ROS_DOMAIN_ID=4 and the agent up?")
        return found

    def _on_status(self, msg: VehicleStatus):
        self.status = msg
        nav = NAV_STATE.get(msg.nav_state, str(msg.nav_state))
        intent = NAV_STATE.get(msg.nav_state_user_intention, str(msg.nav_state_user_intention))
        if nav != self.last_nav:
            self.get_logger().info(
                f"NAV_STATE -> {nav}  (user_intention={intent} "
                f"armed={ARMING_STATE.get(msg.arming_state, msg.arming_state)} "
                f"failsafe={msg.failsafe} user_took_over={msg.failsafe_and_user_took_over} "
                f"preflight_pass={msg.pre_flight_checks_pass})")
            self.last_nav = nav
        if msg.failsafe_and_user_took_over != self.last_took_over:
            if msg.failsafe_and_user_took_over:
                self.get_logger().warn(
                    "RC OVERRIDE active (failsafe_and_user_took_over=true) -- PX4 will NOT return to "
                    "OFFBOARD until this clears (return RC mode switch / re-issue offboard after override ends).")
            self.last_took_over = msg.failsafe_and_user_took_over

    def _on_lpos(self, msg: VehicleLocalPosition):
        self.lpos = msg

    def _on_heartbeat(self, _msg: OffboardControlMode):
        self.hb_times.append(time.monotonic())

    def _hb_rate(self) -> float:
        now = time.monotonic()
        self.hb_times = [t for t in self.hb_times if now - t <= 2.0]
        return len(self.hb_times) / 2.0

    def _pos_blockers(self) -> list[str]:
        """Required-for-offboard estimate flags that are NOT in the good state."""
        if self.lpos is None:
            return ["<no vehicle_local_position>"]
        bad = []
        for f in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control"):
            if not getattr(self.lpos, f, False):
                bad.append(f"!{f}")
        if getattr(self.lpos, "dead_reckoning", False):
            bad.append("dead_reckoning")
        return bad

    def _pos_summary(self) -> str:
        if self.lpos is None:
            return "lpos=none"
        return " ".join(
            f"{f}={int(getattr(self.lpos, f, False))}" for f in POS_FLAGS)

    def _tick(self):
        hb = self._hb_rate()
        if self.status is None:
            self.get_logger().warn(f"no vehicle_status yet (offboard_hb={hb:.0f}Hz)")
            return
        s = self.status
        nav = NAV_STATE.get(s.nav_state, str(s.nav_state))
        intent = NAV_STATE.get(s.nav_state_user_intention, str(s.nav_state_user_intention))
        posblk = ",".join(self._pos_blockers()) or "none"
        self.get_logger().info(
            f"nav={nav} intent={intent} armed={ARMING_STATE.get(s.arming_state, s.arming_state)} "
            f"failsafe={s.failsafe} rc_override={s.failsafe_and_user_took_over} "
            f"preflight={s.pre_flight_checks_pass} offboard_hb={hb:.0f}Hz pos_blockers=[{posblk}]")

        # The decisive correlation: the stack asked for OFFBOARD but PX4 is not in it.
        if s.nav_state_user_intention == NAV_STATE_OFFBOARD and s.nav_state != NAV_STATE_OFFBOARD:
            reasons = []
            if s.failsafe_and_user_took_over:
                reasons.append("RC override active (return mode switch / sticks)")
            blk = self._pos_blockers()
            if blk and blk != ["<no vehicle_local_position>"]:
                reasons.append(f"invalid estimate: {blk}")
            if hb < 2.0:
                reasons.append(f"offboard heartbeat low ({hb:.0f}Hz)")
            if not s.pre_flight_checks_pass:
                reasons.append("pre-flight checks failing")
            self.get_logger().error(
                "OFFBOARD requested but NOT active. Likely cause(s): "
                + ("; ".join(reasons) if reasons else
                   "none of the tracked preconditions -- check RC mode switch position / arming"))
            self.get_logger().error("  local_position flags: " + self._pos_summary())

        if hb < 2.0:
            self.get_logger().warn(
                f"offboard heartbeat below 2Hz ({hb:.0f}Hz) -- PX4 will refuse/drop OFFBOARD")


def main():
    parser = argparse.ArgumentParser(description="Diagnose PX4 offboard-mode entry failures")
    parser.add_argument(
        "--px4-namespace", default="",
        help="Namespace prefix of /fmu topics (default: root, as bridged on this airframe)")
    args = parser.parse_args()

    rclpy.init()
    node = OffboardDiag(args.px4_namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
