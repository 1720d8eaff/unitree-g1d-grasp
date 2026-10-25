#!/usr/bin/env bash
source /opt/ros/foxy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
PREFIX="$HOME/ros2_ws/install/g1_d_trajectory_bridge"
PY_SITE="$PREFIX/lib/python3.8/site-packages"
export AMENT_PREFIX_PATH="$PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PYTHONPATH="$PY_SITE:$HOME/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$PREFIX/lib/g1_d_trajectory_bridge${PATH:+:$PATH}"
