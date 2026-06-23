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
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

# ── Persist ROS + bridge logs to a mounted directory ──────────
# ROS_LOG_DIR is normally set per-role by the compose file to a path under the
# bind-mounted workspace (e.g. ${ROS_WS}/logs/aircraft4), so node logs (rosout,
# per-node stdout/stderr) survive container removal and are available on the host
# for post-incident analysis. Fall back to a hostname-scoped dir if unset.
export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROS_WS}/logs/$(hostname)}"
mkdir -p "${ROS_LOG_DIR}" 2>/dev/null || true
echo "[entrypoint] ROS_LOG_DIR=${ROS_LOG_DIR}"

# ── Enable multicast on loopback for DDS discovery ───────────
# CycloneDDS needs multicast on lo when ROS_LOCALHOST_ONLY=1;
# without this, Zenoh bridge DDS participants can't discover ROS nodes.
sudo ip link set lo multicast on 2>/dev/null || true

# ── Start Micro-XRCE-DDS Agent if requested ───────────────────
if [ "${START_XRCE_AGENT:-false}" = "true" ]; then
    if [ -n "${XRCE_DEVICE}" ]; then
        XRCE_BAUD="${XRCE_BAUD:-921600}"
        echo "[entrypoint] Setting ${XRCE_DEVICE} baud rate to ${XRCE_BAUD}..."
        sudo stty -F "${XRCE_DEVICE}" "${XRCE_BAUD}" || echo "[entrypoint] WARNING: stty failed for ${XRCE_DEVICE}, continuing anyway..."
        echo "[entrypoint] Starting Micro-XRCE-DDS Agent on serial ${XRCE_DEVICE} @ ${XRCE_BAUD}..."
        sudo MicroXRCEAgent serial --dev "${XRCE_DEVICE}" -b "${XRCE_BAUD}" &
    else
        XRCE_PORT="${XRCE_PORT:-8888}"
        echo "[entrypoint] Starting Micro-XRCE-DDS Agent on UDP port ${XRCE_PORT}..."
        sudo MicroXRCEAgent udp4 -p "${XRCE_PORT}" &
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
    ZENOH_LOG_FILE="${ROS_LOG_DIR}/zenoh_bridge_$(date +%Y%m%d-%H%M%S).log"
    echo "[entrypoint] Starting zenoh-bridge-ros2dds (port=${ZENOH_PORT}), logging to ${ZENOH_LOG_FILE}..."
    # tee keeps the bridge output on the container stdout (docker logs) AND on a
    # persisted file so the Zenoh route/discovery history survives a restart.
    zenoh-bridge-ros2dds -c "${ZENOH_RUNTIME_CONFIG}" 2>&1 | tee -a "${ZENOH_LOG_FILE}" &
    sleep 1
fi

# ── Execute the command passed to the container ────────────────
exec "$@"
