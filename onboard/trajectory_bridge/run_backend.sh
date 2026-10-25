#!/usr/bin/env bash
set -euo pipefail
source "$HOME/g1_trajectory_bridge_env.sh"
if pgrep -af '[t]rajectory_backend' >/dev/null; then
  echo "trajectory_backend 已在运行" >&2
  exit 1
fi
exec /usr/bin/python3 -m g1_d_trajectory_bridge.trajectory_backend
