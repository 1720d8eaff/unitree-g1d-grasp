#!/usr/bin/env bash
set -euo pipefail
source "$HOME/g1_trajectory_bridge_env.sh"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec ros2 run g1_d_trajectory_bridge trajectory_frontend
