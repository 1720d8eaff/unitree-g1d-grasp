#!/usr/bin/env bash
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${HOME}/g1_ws/src/g1_d_moveit_bridge"
BACKUP_ROOT="${HOME}/g1_ws_backups"
STAMP="$(date +%Y%m%d_%H%M%S)"

python3 -m py_compile \
  "${BUNDLE_DIR}/g1_d_moveit_bridge/trajectory_action_relay.py" \
  "${BUNDLE_DIR}/g1_d_moveit_bridge/trajectory_action_relay_dry_run.py" \
  "${BUNDLE_DIR}/setup.py"

mkdir -p "${BACKUP_ROOT}"
if [[ -e "${TARGET}" ]]; then
  BACKUP="${BACKUP_ROOT}/g1_d_moveit_bridge.backup_${STAMP}"
  mv "${TARGET}" "${BACKUP}"
  echo "已将现有包移出 src 并备份到：${BACKUP}"
fi

mkdir -p "${TARGET}"
cp -a "${BUNDLE_DIR}/." "${TARGET}/"
find "${TARGET}" -type d -name '__pycache__' -prune -exec rm -rf {} +
find "${TARGET}" -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete

echo "笔记本端源码安装完成：${TARGET}"
echo "尚未编译，也没有启动任何节点。"
