#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SERVICE="${1:-}"

if command -v conda >/dev/null 2>&1; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda deactivate 2>/dev/null || true
  conda deactivate 2>/dev/null || true
fi

source /opt/ros/foxy/setup.bash
source /home/unitree/ros2_ws/install/setup.bash
source /home/unitree/g1_trajectory_bridge_env.sh
source /home/unitree/cyclonedds_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

case "$SERVICE" in
  driver)
    # Run the project-pinned driver source directly while reusing the board's
    # installed Foxy message packages. This avoids silently selecting a stale
    # rh56h1_udp_driver entry point from ros2_ws.
    export PYTHONPATH="$PROJECT_DIR/hand/onboard/rh56h1_udp_driver:${PYTHONPATH:-}"
    exec python3 -u \
      "$PROJECT_DIR/hand/onboard/rh56h1_udp_driver/rh56h1_udp_driver/driver_node.py"
    ;;
  mapper)
    exec python3 -u \
      "$PROJECT_DIR/hand/onboard/rh56h1_virtual_gripper_mapper_v5.py"
    ;;
  controller)
    exec python3 -u \
      "$PROJECT_DIR/hand/onboard/rh56h1_virtual_gripper_controller_v4.py"
    ;;
  *)
    echo "usage: $0 {driver|mapper|controller}" >&2
    exit 2
    ;;
esac
