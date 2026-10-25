#!/usr/bin/env bash

set -Eeo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="/home/yii/g1_ws/install/setup.bash"

CYCLONEDDS_CONFIG="/home/yii/cyclonedds_ws/cyclonedds.xml"

ROBOT_IP="192.168.123.164"
ETHERNET_INTERFACE="enp8s0"


echo "========================================"
echo "   启动 G1-D MoveIt 2"
echo "========================================"


# 检查必需文件
for required_file in \
    "${ROS_SETUP}" \
    "${WORKSPACE_SETUP}" \
    "${CYCLONEDDS_CONFIG}"
do
    if [[ ! -e "${required_file}" ]]; then
        echo "错误：缺少文件：${required_file}"
        exit 1
    fi
done


# 检查有线网卡
if ! ip link show "${ETHERNET_INTERFACE}" >/dev/null 2>&1; then
    echo "错误：未找到网卡 ${ETHERNET_INTERFACE}"
    exit 1
fi

if ! ip -4 addr show "${ETHERNET_INTERFACE}" | grep -q "192\.168\.123\."; then
    echo "错误：${ETHERNET_INTERFACE} 没有 192.168.123.x 地址。"
    exit 1
fi


# 检查机器人网络
echo "[1/3] 检查 G1-D 网络连接……"

if ! ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
    echo "错误：无法连接 G1-D 板载机 ${ROBOT_IP}"
    exit 1
fi


# 防止重复启动
if pgrep -x move_group >/dev/null 2>&1 || \
   pgrep -x rviz2 >/dev/null 2>&1 || \
   pgrep -x robot_state_publisher >/dev/null 2>&1
then
    echo "错误：检测到 MoveIt、RViz 或 robot_state_publisher 已在运行。"
    echo "请先关闭旧的启动终端，再重新执行。"
    exit 1
fi


# 加载 ROS 2 Humble 和工作空间
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"


# 使用和板载机一致的 CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_CONFIG}"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0


echo "[2/3] 检查真实关节状态……"

# 防止 ROS 2 daemon 仍运行在 Fast DDS 环境
ros2 daemon stop >/dev/null 2>&1 || true


# 最多等待 10 秒，确认笔记本确实收到了真实状态
if ! timeout 10 ros2 topic echo /joint_states --once \
    >/tmp/g1_joint_state_startup_check.yaml 2>/dev/null
then
    echo "错误：10 秒内没有收到 /joint_states。"
    echo
    echo "请确认："
    echo "1. 板载机已经运行 start_g1_state"
    echo "2. 网线连接正常"
    echo "3. G1-D 已开机"
    exit 1
fi


echo "[3/3] 启动 MoveIt、RViz 和 TF……"
echo
echo "关闭时直接按 Ctrl+C。"
echo


# 使用 exec，让 Ctrl+C 直接由 ros2 launch 接管并关闭所有子节点
exec ros2 launch g1_d_moveit_config real_robot.launch.py
