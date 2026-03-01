#!/usr/bin/env bash
# ╔══════════════════════════════════════════════════════════════╗
# ║  Common entrypoint for all ros2-px4-docker images           ║
# ╚══════════════════════════════════════════════════════════════╝
set -e

# ── Source ROS2 + PX4 workspaces ───────────────────────────────
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
ROS_WS="${ROS_WS:-/ros2_ws}"

if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

# ── Set ROS_DOMAIN_ID from DRONE_ID ───────────────────────────
export ROS_DOMAIN_ID=${DRONE_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}

# ── Start Micro-XRCE-DDS Agent if requested ───────────────────
if [ "${START_XRCE_AGENT:-false}" = "true" ]; then
    if [ -n "${XRCE_DEVICE}" ]; then
        XRCE_BAUD="${XRCE_BAUD:-921600}"
        echo "[entrypoint] Setting ${XRCE_DEVICE} baud rate to ${XRCE_BAUD}..."
        sudo stty -F "${XRCE_DEVICE}" "${XRCE_BAUD}" || echo "[entrypoint] WARNING: stty failed for ${XRCE_DEVICE}, continuing anyway..."
        echo "[entrypoint] Starting Micro-XRCE-DDS Agent on serial ${XRCE_DEVICE} @ ${XRCE_BAUD}..."
        MicroXRCEAgent serial --dev "${XRCE_DEVICE}" -b "${XRCE_BAUD}" &
    else
        XRCE_PORT="${XRCE_PORT:-8888}"
        echo "[entrypoint] Starting Micro-XRCE-DDS Agent on UDP port ${XRCE_PORT}..."
        MicroXRCEAgent udp4 -p "${XRCE_PORT}" &
    fi
    sleep 1
fi

# ── Start Zenoh bridge if configured ─────────────────────────
if [ -n "${ZENOH_BRIDGE_CONFIG}" ]; then
    ZENOH_PORT="${ZENOH_PORT:-7447}"
    ZENOH_RUNTIME_CONFIG="/tmp/zenoh_bridge.json5"
    echo "[entrypoint] Templating Zenoh config: ${ZENOH_BRIDGE_CONFIG} → ${ZENOH_RUNTIME_CONFIG}"
    sed -e "s/__DRONE_ID__/${DRONE_ID:-0}/g" \
        -e "s/__ZENOH_PORT__/${ZENOH_PORT}/g" \
        "${ZENOH_BRIDGE_CONFIG}" > "${ZENOH_RUNTIME_CONFIG}"
    echo "[entrypoint] Starting zenoh-bridge-ros2dds (port=${ZENOH_PORT})..."
    zenoh-bridge-ros2dds -c "${ZENOH_RUNTIME_CONFIG}" &
    sleep 1
fi

# ── Execute the command passed to the container ────────────────
exec "$@"
