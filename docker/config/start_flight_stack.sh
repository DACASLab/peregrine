#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

if [ ! -f install/setup.bash ]; then
  echo "[start_flight_stack] Workspace not built. Run 'make dev-jetson' and 'colcon build' first."
  exit 0
fi

source install/setup.bash

ros2 launch peregrine_bringup bt_mission.launch.py \
  start_microxrce_agent:=false \
  ros_domain_id:=${DRONE_ID:-1} \
  ros_localhost_only:=${ROS_LOCALHOST_ONLY:-1} \
  uav_namespace:=/uav${DRONE_ID:-1} \
  target_system_id:=${DRONE_ID:-1}
