#!/usr/bin/env bash
# G1-D 板载机一键启动：状态链 + trajectory_backend + trajectory_frontend
# 设计原则：启动后保持 DISARMED；绝不自动启用真实轨迹控制。

set -Eeo pipefail

ROS_SETUP="/opt/ros/foxy/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"
BACKEND_SOCKET="/tmp/g1_d_trajectory_backend.sock"

# 与当前已验证板载环境保持一致
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_HOME="/home/unitree/cyclonedds_ws/install/cyclonedds"
export CYCLONEDDS_URI="/home/unitree/cyclonedds_ws/cyclonedds.xml"

STATE_PID=""
BACKEND_PID=""
FRONTEND_PID=""
CLEANING_UP=0

source "$ROS_SETUP"
source "$WS_SETUP"

log() {
  printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

stop_pid() {
  local pid="${1:-}"
  local name="${2:-process}"
  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  log "停止 $name (PID=$pid) ..."
  kill -INT "$pid" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM "$pid" 2>/dev/null || true
}

cleanup() {
  (( CLEANING_UP == 0 )) || return 0
  CLEANING_UP=1
  trap - INT TERM EXIT

  echo
  echo "========================================"
  echo "正在停止 G1-D 真实控制链..."
  echo "========================================"

  # 若服务存在，先请求 DISARM。不要在 ARMED 状态直接杀 backend。
  if ros2 service list 2>/dev/null | grep -qx '/g1_d/set_trajectory_control'; then
    log "先请求 DISARM ..."
    timeout 10s ros2 service call \
      /g1_d/set_trajectory_control \
      std_srvs/srv/SetBool \
      "{data: false}" || true
    sleep 1
  fi

  stop_pid "$FRONTEND_PID" "trajectory_frontend"
  stop_pid "$BACKEND_PID" "trajectory_backend"
  stop_pid "$STATE_PID" "start_g1_state"

  log "板载控制链已停止"
}

trap cleanup INT TERM EXIT

# 防止重复实例。这里只检查，不自动杀已有进程。
if pgrep -af '[t]rajectory_backend|[t]rajectory_frontend|[u]nitree_reader.py|[s]tate_bridge_node' >/tmp/g1_onboard_existing.$$ 2>/dev/null; then
  echo "检测到已有相关进程，拒绝重复启动："
  cat /tmp/g1_onboard_existing.$$
  rm -f /tmp/g1_onboard_existing.$$
  echo
  echo "请先确认旧链路已安全 DISARM 并停止，再重新运行本脚本。"
  exit 1
fi
rm -f /tmp/g1_onboard_existing.$$

# 启动前 /lowcmd 不允许有外部 publisher。
LOWCMD_INFO="$(ros2 topic info /lowcmd 2>/dev/null || true)"
LOWCMD_PUB="$(printf '%s\n' "$LOWCMD_INFO" | awk -F': ' '/Publisher count:/ {print $2; exit}')"
LOWCMD_SUB="$(printf '%s\n' "$LOWCMD_INFO" | awk -F': ' '/Subscription count:/ {print $2; exit}')"

if [[ "$LOWCMD_PUB" != "0" ]]; then
  echo "启动前安全检查失败：/lowcmd Publisher count 不是 0。"
  printf '%s\n' "$LOWCMD_INFO"
  exit 1
fi

if [[ -n "$LOWCMD_SUB" && "$LOWCMD_SUB" != "1" ]]; then
  echo "启动前安全检查失败：/lowcmd Subscription count 不是 1。"
  printf '%s\n' "$LOWCMD_INFO"
  exit 1
fi

rm -f "$BACKEND_SOCKET"

echo "========================================"
echo "启动 G1-D 板载真实控制链"
echo "状态：默认 DISARMED，不会自动控制真机"
echo "========================================"

log "[1/3] 启动真实状态链 start_g1_state ..."
# start_g1_state 是已验证的交互 shell 命令；用交互 bash 调用以兼容函数/alias。
bash -ic 'start_g1_state' &
STATE_PID=$!

# 等待 /joint_states publisher 出现
for _ in {1..80}; do
  JS_PUB="$(ros2 topic info /joint_states 2>/dev/null | awk -F': ' '/Publisher count:/ {print $2; exit}')"
  [[ "$JS_PUB" == "1" ]] && break
  kill -0 "$STATE_PID" 2>/dev/null || {
    echo "start_g1_state 提前退出。"
    exit 1
  }
  sleep 0.25
done

JS_PUB="$(ros2 topic info /joint_states 2>/dev/null | awk -F': ' '/Publisher count:/ {print $2; exit}')"
if [[ "$JS_PUB" != "1" ]]; then
  echo "等待 /joint_states 单一 publisher 超时。"
  exit 1
fi

log "[2/3] 启动 trajectory_backend ..."
ros2 run g1_d_trajectory_bridge trajectory_backend &
BACKEND_PID=$!

for _ in {1..80}; do
  [[ -S "$BACKEND_SOCKET" ]] && break
  kill -0 "$BACKEND_PID" 2>/dev/null || {
    echo "trajectory_backend 提前退出。"
    exit 1
  }
  sleep 0.25
done

if [[ ! -S "$BACKEND_SOCKET" ]]; then
  echo "等待 backend Unix socket 超时：$BACKEND_SOCKET"
  exit 1
fi

log "[3/3] 启动 trajectory_frontend ..."
ros2 run g1_d_trajectory_bridge trajectory_frontend &
FRONTEND_PID=$!

# 等待 command/status 端点发现
for _ in {1..80}; do
  CMD_SUB="$(ros2 topic info /g1_d/trajectory_command 2>/dev/null | awk -F': ' '/Subscription count:/ {print $2; exit}')"
  STATUS_PUB="$(ros2 topic info /g1_d/trajectory_status 2>/dev/null | awk -F': ' '/Publisher count:/ {print $2; exit}')"
  [[ "$CMD_SUB" == "1" && "$STATUS_PUB" == "1" ]] && break
  kill -0 "$FRONTEND_PID" 2>/dev/null || {
    echo "trajectory_frontend 提前退出。"
    exit 1
  }
  sleep 0.25
done

echo
echo "========================================"
echo "G1-D 板载控制链启动完成"
echo "  /joint_states publisher : ${JS_PUB:-?}"
echo "  trajectory backend      : PID=$BACKEND_PID"
echo "  trajectory frontend     : PID=$FRONTEND_PID"
echo "  当前状态                : DISARMED"
echo "========================================"
echo "保持本终端运行。"
echo "真实执行前仍需手动 ARM："
echo "  ros2 service call /g1_d/set_trajectory_control std_srvs/srv/SetBool \"{data: true}\""
echo "结束时按 Ctrl+C；脚本会先尝试 DISARM，再停止各进程。"
echo

# 任一核心子进程异常退出就触发整体清理。
wait -n "$STATE_PID" "$BACKEND_PID" "$FRONTEND_PID"
EXIT_CODE=$?
echo "某个核心进程已退出 (code=$EXIT_CODE)，开始安全清理。"
exit "$EXIT_CODE"
