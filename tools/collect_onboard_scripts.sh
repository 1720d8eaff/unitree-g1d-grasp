#!/usr/bin/env bash
# Export only project-specific board scripts/sources. Run manually on the board.
# This script never ARM/DISARM, publishes no DDS command, and does not stop any process.

set -Eeo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date '+%Y%m%d-%H%M%S')"
OUTPUT_DIR="$PROJECT_ROOT/runs"
OUTPUT="$OUTPUT_DIR/onboard_script_export_${STAMP}.tar.gz"
STAGE="$(mktemp -d /tmp/g1d-script-export.XXXXXX)"
trap 'rm -rf -- "$STAGE"' EXIT

mkdir -p "$OUTPUT_DIR" "$STAGE/files"

copy_file() {
  local source="$1"
  local relative="$2"
  if [[ -f "$source" ]]; then
    mkdir -p "$STAGE/files/$(dirname -- "$relative")"
    cp -a -- "$source" "$STAGE/files/$relative"
    printf 'COLLECTED %s\n' "$source"
  else
    printf 'MISSING   %s\n' "$source"
  fi
}

copy_tree() {
  local source="$1"
  local relative="$2"
  if [[ -d "$source" ]]; then
    mkdir -p "$STAGE/files/$(dirname -- "$relative")"
    cp -a -- "$source" "$STAGE/files/$relative"
    printf 'COLLECTED %s\n' "$source"
  else
    printf 'MISSING   %s\n' "$source"
  fi
}

copy_file /usr/local/bin/start_g1_control startup/start_g1_control
copy_file /home/unitree/.local/bin/start_g1_state startup/start_g1_state
copy_file /home/unitree/g1_trajectory_bridge_env.sh startup/g1_trajectory_bridge_env.sh
copy_file /home/unitree/g1_experiments/state_bridge_test/unitree_reader.py state/unitree_reader.py
copy_tree /home/unitree/ros2_ws/src/g1_d_state_bridge state/g1_d_state_bridge
copy_file /home/unitree/unitree_sdk2/example/g1/g1d/g1d_height_15cm.cpp height/g1d_height_15cm.cpp
copy_file /home/unitree/rh56h1_virtual_gripper_controller_v4.py hand/rh56h1_virtual_gripper_controller_v4.py
copy_file /home/unitree/rh56h1_virtual_gripper_mapper_v5.py hand/rh56h1_virtual_gripper_mapper_v5.py
copy_tree /home/unitree/ros2_ws/src/rh56h1_udp_driver hand/rh56h1_udp_driver

(
  cd "$STAGE/files"
  find . -type f -print0 | sort -z | xargs -0 sha256sum > "$STAGE/SHA256SUMS"
)

tar -czf "$OUTPUT" -C "$STAGE" files SHA256SUMS
printf 'WROTE %s\n' "$OUTPUT"
printf 'This export only reads source files; it did not change robot state.\n'
