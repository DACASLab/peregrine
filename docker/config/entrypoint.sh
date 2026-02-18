#!/usr/bin/env bash
# ╔══════════════════════════════════════════════════════════════╗
# ║  Common entrypoint for all ros2-px4-docker images           ║
# ╚══════════════════════════════════════════════════════════════╝
set -e

# ── Source ROS2 + PX4 workspaces ───────────────────────────────
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
ROS_WS="${ROS_WS:-/ros2_ws}"

if [ -f /opt/px4_ws/install/setup.bash ]; then
    source /opt/px4_ws/install/setup.bash
fi

if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

# ── Set ROS_DOMAIN_ID from DRONE_ID ───────────────────────────
export ROS_DOMAIN_ID=${DRONE_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}

# ── Start Micro-XRCE-DDS Agent if requested ───────────────────
if [ "${START_XRCE_AGENT:-false}" = "true" ]; then
    echo "[entrypoint] Starting Micro-XRCE-DDS Agent on default UDP port..."
    MicroXRCEAgent udp4 -p 8888 &
fi

# ── Execute the command passed to the container ────────────────
exec "$@"
