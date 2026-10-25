#!/usr/bin/env bash
# G1-D 笔记本一键启动：唯一 trajectory_action_relay + MoveIt/RViz
# 关键：relay 与 move_group 使用完全一致的 CycloneDDS 环境。

set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/g1_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="/home/yii/cyclonedds_ws/cyclonedds.xml"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

RELAY_PID=""
MOVEIT_PID=""
CLEANING_UP=0

source "$ROS_SETUP"
source "$WS_SETUP"

log() {
  printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

cleanup() {
  (( CLEANING_UP == 0 )) || return 0
  CLEANING_UP=1
  trap - INT TERM EXIT

  echo
  echo "========================================"
  echo "正在停止 G1-D 笔记本控制链..."
  echo "========================================"

  if [[ -n "$MOVEIT_PID" ]] && kill -0 "$MOVEIT_PID" 2>/dev/null; then
    kill -INT "$MOVEIT_PID" 2>/dev/null || true
    wait "$MOVEIT_PID" 2>/dev/null || true
  fi

  if [[ -n "$RELAY_PID" ]] && kill -0 "$RELAY_PID" 2>/dev/null; then
    kill -INT "$RELAY_PID" 2>/dev/null || true
    wait "$RELAY_PID" 2>/dev/null || true
  fi

  log "笔记本控制链已停止"
}

trap cleanup INT TERM EXIT

# 不允许重复 relay。
EXISTING_RELAY="$(ros2 node list 2>/dev/null | grep -x '/g1_d_trajectory_action_relay' | wc -l | tr -d ' ' || true)"
if [[ "$EXISTING_RELAY" != "0" ]]; then
  echo "检测到已有 /g1_d_trajectory_action_relay 节点，拒绝重复启动。"
  echo "当前数量：$EXISTING_RELAY"
  exit 1
fi

# 不自动清理已有 move_group，避免误杀其他调试会话。
if pgrep -f '/moveit_ros_move_group/move_group' >/dev/null 2>&1; then
  echo "检测到已有 move_group，拒绝重复启动。"
  echo "请先停止旧 MoveIt，再运行本脚本。"
  exit 1
fi

echo "========================================"
echo "启动 G1-D 笔记本 MoveIt 真实控制链"
echo "DDS: CycloneDDS / ROS_DOMAIN_ID=0"
echo "========================================"

log "[1/2] 启动唯一 trajectory_action_relay ..."
ros2 run g1_d_moveit_bridge trajectory_action_relay &
RELAY_PID=$!

# 等待 relay ROS 节点与 right_arm Action server 出现。
for _ in {1..80}; do
  RELAY_COUNT="$(ros2 node list 2>/dev/null | grep -x '/g1_d_trajectory_action_relay' | wc -l | tr -d ' ' || true)"
  ACTION_OK="$(ros2 action info /right_arm_controller/follow_joint_trajectory 2>/dev/null | awk -F': ' '/Action servers:/ {print $2; exit}' || true)"
  [[ "$RELAY_COUNT" == "1" && "$ACTION_OK" == "1" ]] && break
  kill -0 "$RELAY_PID" 2>/dev/null || {
    echo "trajectory_action_relay 提前退出。"
    exit 1
  }
  sleep 0.25
done

RELAY_COUNT="$(ros2 node list 2>/dev/null | grep -x '/g1_d_trajectory_action_relay' | wc -l | tr -d ' ' || true)"
if [[ "$RELAY_COUNT" != "1" ]]; then
  echo "relay 启动检查失败：ROS 节点数量=$RELAY_COUNT"
  exit 1
fi

log "[2/2] 启动 MoveIt/RViz（调用项目内 start_g1_moveit.sh）..."
"$SCRIPT_DIR/start_g1_moveit.sh" &
MOVEIT_PID=$!

# 等待 move_group 与 MoveIt Action client 出现。
for _ in {1..120}; do
  MOVE_GROUP_COUNT="$(ros2 node list 2>/dev/null | grep -x '/move_group' | wc -l | tr -d ' ' || true)"
  ACTION_CLIENTS="$(ros2 action info /right_arm_controller/follow_joint_trajectory 2>/dev/null | awk -F': ' '/Action clients:/ {print $2; exit}' || true)"
  [[ "$MOVE_GROUP_COUNT" == "1" && "$ACTION_CLIENTS" == "1" ]] && break
  kill -0 "$MOVEIT_PID" 2>/dev/null || {
    echo "start_g1_moveit 提前退出。"
    exit 1
  }
  sleep 0.5
done

# 最终拓扑检查
RELAY_COUNT="$(ros2 node list 2>/dev/null | grep -x '/g1_d_trajectory_action_relay' | wc -l | tr -d ' ' || true)"
ACTION_INFO="$(ros2 action info /right_arm_controller/follow_joint_trajectory 2>/dev/null || true)"
ACTION_CLIENTS="$(printf '%s\n' "$ACTION_INFO" | awk -F': ' '/Action clients:/ {print $2; exit}')"
ACTION_SERVERS="$(printf '%s\n' "$ACTION_INFO" | awk -F': ' '/Action servers:/ {print $2; exit}')"
CMD_INFO="$(ros2 topic info /g1_d/trajectory_command 2>/dev/null || true)"
STATUS_INFO="$(ros2 topic info /g1_d/trajectory_status 2>/dev/null || true)"
CMD_PUB="$(printf '%s\n' "$CMD_INFO" | awk -F': ' '/Publisher count:/ {print $2; exit}')"
CMD_SUB="$(printf '%s\n' "$CMD_INFO" | awk -F': ' '/Subscription count:/ {print $2; exit}')"
STATUS_PUB="$(printf '%s\n' "$STATUS_INFO" | awk -F': ' '/Publisher count:/ {print $2; exit}')"
STATUS_SUB="$(printf '%s\n' "$STATUS_INFO" | awk -F': ' '/Subscription count:/ {print $2; exit}')"

echo
echo "========================================"
echo "G1-D 笔记本控制链启动完成"
echo "  relay ROS节点数量       : ${RELAY_COUNT:-?}"
echo "  right_arm Action client : ${ACTION_CLIENTS:-?}"
echo "  right_arm Action server : ${ACTION_SERVERS:-?}"
echo "  trajectory_command      : pub=${CMD_PUB:-?}, sub=${CMD_SUB:-?}"
echo "  trajectory_status       : pub=${STATUS_PUB:-?}, sub=${STATUS_SUB:-?}"
echo "========================================"
echo "保持本终端运行。"
echo "脚本不会自动 ARM 真机。"
echo

if [[ "$RELAY_COUNT" != "1" || "$ACTION_CLIENTS" != "1" || "$ACTION_SERVERS" != "1" ]]; then
  echo "核心拓扑检查未通过，请不要 Execute。"
  exit 1
fi

# 若板载链已启动，则 command/status 应为 1/1；未启动时允许显示 ?/0。
if [[ "$CMD_PUB" != "1" || "$STATUS_SUB" != "1" ]]; then
  echo "relay topic 端点异常，请不要 Execute。"
  exit 1
fi

# MoveIt 会一直运行；任一核心进程退出就清理另一端。
wait -n "$RELAY_PID" "$MOVEIT_PID"
EXIT_CODE=$?
echo "某个核心进程已退出 (code=$EXIT_CODE)，开始清理。"
exit "$EXIT_CODE"
