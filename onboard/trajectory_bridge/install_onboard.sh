#!/usr/bin/env bash
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${HOME}/ros2_ws/src/g1_d_trajectory_bridge"
BACKUP_ROOT="${HOME}/ros2_ws_backups"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ "$(id -u)" -eq 0 ]]; then
  echo "不要使用 root 运行安装脚本" >&2
  exit 1
fi

/usr/bin/python3 -m py_compile \
  "${BUNDLE_DIR}/g1_d_trajectory_bridge/ipc_protocol.py" \
  "${BUNDLE_DIR}/g1_d_trajectory_bridge/trajectory_backend.py" \
  "${BUNDLE_DIR}/g1_d_trajectory_bridge/trajectory_frontend.py" \
  "${BUNDLE_DIR}/g1_d_trajectory_bridge/trajectory_executor_blocked.py" \
  "${BUNDLE_DIR}/g1_d_trajectory_bridge/trajectory_topic_dry_run.py" \
  "${BUNDLE_DIR}/setup.py"

mkdir -p "${BACKUP_ROOT}"
if [[ -e "${TARGET}" ]]; then
  BACKUP="${BACKUP_ROOT}/g1_d_trajectory_bridge.backup_${STAMP}"
  mv "${TARGET}" "${BACKUP}"
  echo "已将现有包移出 src 并备份到：${BACKUP}"
fi

mkdir -p "${TARGET}"
cp -a "${BUNDLE_DIR}/." "${TARGET}/"
find "${TARGET}" -type d -name '__pycache__' -prune -exec rm -rf {} +
find "${TARGET}" -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete

cat > "${HOME}/g1_trajectory_bridge_env.sh" <<'EOF'
#!/usr/bin/env bash
source /opt/ros/foxy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
PREFIX="$HOME/ros2_ws/install/g1_d_trajectory_bridge"
PY_SITE="$PREFIX/lib/python3.8/site-packages"
export AMENT_PREFIX_PATH="$PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PYTHONPATH="$PY_SITE:$HOME/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$PREFIX/lib/g1_d_trajectory_bridge${PATH:+:$PATH}"
EOF
chmod +x "${HOME}/g1_trajectory_bridge_env.sh"

echo "板载双进程桥源码安装完成：${TARGET}"
echo "备份位于工作空间外，不会造成 colcon 重名包。"
echo "尚未编译，也没有启动任何节点或 LowCmd。"
