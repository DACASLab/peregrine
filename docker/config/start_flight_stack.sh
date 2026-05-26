#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
[ -f install/setup.bash ] && source install/setup.bash || true

ros2 launch peregrine_bringup core_stack.launch.py \
  start_microxrce_agent:=false \
  ros_domain_id:=${DRONE_ID:-1} \
  ros_localhost_only:=${ROS_LOCALHOST_ONLY:-1} \
  uav_namespace:=/uav${DRONE_ID:-1} \
  target_system_id:=1
