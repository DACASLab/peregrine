#!/usr/bin/env bash
# ╔══════════════════════════════════════════════════════════════╗
# ║  Common entrypoint for all ros2-px4-docker images           ║
# ╚══════════════════════════════════════════════════════════════╝
set -e

# ── Source ROS2 + PX4 workspaces ───────────────────────────────
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

if [ -f /opt/px4_ws/install/setup.bash ]; then
    source /opt/px4_ws/install/setup.bash
fi

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# ── Set ROS_DOMAIN_ID from DRONE_ID ───────────────────────────
export ROS_DOMAIN_ID=${DRONE_ID:-0}

# ── DDS middleware selection ───────────────────────────────────
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

if [ -f "${CYCLONEDDS_URI}" ]; then
    export CYCLONEDDS_URI
fi

# ── Start Micro-XRCE-DDS Agent if requested ───────────────────
if [ "${START_XRCE_AGENT:-false}" = "true" ]; then
    echo "[entrypoint] Starting Micro-XRCE-DDS Agent on UDP port ${XRCE_DDS_PORT:-8888}..."
    MicroXRCEAgent udp4 -p "${XRCE_DDS_PORT:-8888}" &
fi

# ── Execute the command passed to the container ────────────────
exec "$@"
