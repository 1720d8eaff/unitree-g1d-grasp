#!/usr/bin/env bash

set -Eeo pipefail

ROS_SETUP="/opt/ros/foxy/setup.bash"
WORKSPACE_SETUP="/home/unitree/ros2_ws/install/setup.bash"

READER_SCRIPT="/home/unitree/g1_experiments/state_bridge_test/unitree_reader.py"
BRIDGE_EXEC="/home/unitree/ros2_ws/install/g1_d_state_bridge/lib/g1_d_state_bridge/state_bridge_node"

STATE_FILE="/tmp/g1_lowstate.json"
TEMP_FILE="/tmp/g1_lowstate.json.tmp"

CYCLONEDDS_CONFIG="/home/unitree/cyclonedds_ws/cyclonedds.xml"
UNITREE_SDK="/home/unitree/unitree_sdk2_python"

# unitree_reader.py 目前已验证可在 tv 环境中运行
TV_PYTHON="/home/unitree/miniconda3/envs/tv/bin/python3"

READER_PID=""
BRIDGE_PID=""


cleanup() {
    trap - INT TERM EXIT

    echo
    echo "正在停止 G1-D 状态链路……"

    if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        kill "${BRIDGE_PID}" 2>/dev/null || true
        wait "${BRIDGE_PID}" 2>/dev/null || true
    fi

    if [[ -n "${READER_PID}" ]] && kill -0 "${READER_PID}" 2>/dev/null; then
        kill "${READER_PID}" 2>/dev/null || true
        wait "${READER_PID}" 2>/dev/null || true
    fi

    echo "G1-D 状态链路已停止。"
}


trap cleanup INT TERM EXIT


echo "========================================"
echo "   启动 G1-D 真实状态链路"
echo "========================================"


# 检查必需文件
for required_file in \
    "${ROS_SETUP}" \
    "${WORKSPACE_SETUP}" \
    "${READER_SCRIPT}" \
    "${BRIDGE_EXEC}" \
    "${CYCLONEDDS_CONFIG}"
do
    if [[ ! -e "${required_file}" ]]; then
        echo "错误：缺少文件：${required_file}"
        exit 1
    fi
done


# 防止重复启动
if pgrep -f "${READER_SCRIPT}" >/dev/null 2>&1; then
    echo "错误：unitree_reader.py 已经在运行。"
    pgrep -af "${READER_SCRIPT}"
    exit 1
fi

if pgrep -f "${BRIDGE_EXEC}" >/dev/null 2>&1; then
    echo "错误：g1_d_state_bridge 已经在运行。"
    pgrep -af "${BRIDGE_EXEC}"
    exit 1
fi


# 加载 ROS 2 Foxy 和工作空间
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
source "/home/unitree/cyclonedds_ws/install/setup.bash"


# DDS 和 Unitree SDK 环境
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_CONFIG}"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export PYTHONPATH="${UNITREE_SDK}:${PYTHONPATH:-}"


# 选择 reader 使用的 Python
if [[ -x "${TV_PYTHON}" ]]; then
    READER_PYTHON="${TV_PYTHON}"
else
    READER_PYTHON="/usr/bin/python3"
fi


# 删除上次残留的旧状态文件，避免发布旧姿态
rm -f "${STATE_FILE}" "${TEMP_FILE}"


echo "[1/2] 启动 Unitree LowState reader……"

"${READER_PYTHON}" "${READER_SCRIPT}" &
READER_PID=$!


# 等待 reader 生成新 JSON
reader_ready=false

for _ in $(seq 1 50); do
    if ! kill -0 "${READER_PID}" 2>/dev/null; then
        echo "错误：unitree_reader.py 启动后异常退出。"
        exit 1
    fi

    if [[ -s "${STATE_FILE}" ]]; then
        reader_ready=true
        break
    fi

    sleep 0.1
done


if [[ "${reader_ready}" != true ]]; then
    echo "错误：5 秒内没有生成 ${STATE_FILE}"
    exit 1
fi


echo "[2/2] 启动 ROS 2 JointState bridge……"

"${BRIDGE_EXEC}" &
BRIDGE_PID=$!

sleep 1


if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    echo "错误：g1_d_state_bridge 启动失败。"
    exit 1
fi


echo
echo "========================================"
echo "G1-D 状态链路启动成功"
echo
echo "LowState → JSON → /joint_states"
echo "按 Ctrl+C 可同时停止 reader 和 bridge"
echo "========================================"


# 只要任意一个子进程退出，就结束整套程序
while true; do
    if ! kill -0 "${READER_PID}" 2>/dev/null; then
        echo "错误：unitree_reader.py 已退出。"
        exit 1
    fi

    if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        echo "错误：g1_d_state_bridge 已退出。"
        exit 1
    fi

    sleep 1
done
