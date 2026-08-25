#!/usr/bin/env bash
# =============================================================================
# tool/flat_sim_teleop/run_teleop.sh —— 启动键盘遥控（需 flat_sim 已在运行）
# -----------------------------------------------------------------------------
# 用法：
#   ./run_teleop.sh              默认控制 mycar
#   ./run_teleop.sh --robot foo  控制名为 foo 的机器人
#   ./run_teleop.sh -h|--help
# =============================================================================
set -euo pipefail

TOOL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$(dirname "${TOOL}")")"
ROS_INSTALL="${REPO}/custom_mini_install"
PREFIX="${TOOL}/install"
NODE="${PREFIX}/bin/flat_sim_teleop"
ROBOT="mycar"

usage() {
  cat <<USAGE
用法: $0 [--robot <name> | -h | --help]

  （无参数）       键盘遥控默认机器人 mycar（话题 /mycar/cmd_vel）
  --robot <name>   指定 flat_sim 世界里的机器人名
  -h, --help       显示本说明

按键：↑ 前进  ↓ 后退  ← 逆时针  → 顺时针  q/Ctrl+C 退出
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot)
      [[ $# -ge 2 ]] || { echo "缺少 --robot 参数值" >&2; exit 1; }
      ROBOT="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 1 ;;
  esac
done

[[ -d "${ROS_INSTALL}" ]] || { echo "缺少隔离 ROS（${ROS_INSTALL}），请先 ./custom_mini.sh build" >&2; exit 1; }
[[ -x "${NODE}" ]] || { echo "缺少 ${NODE}，请先 ./tool/flat_sim_teleop/build.sh" >&2; exit 1; }

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

if [[ -f "${ROS_INSTALL}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROS_INSTALL}/setup.bash"
fi

export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}"

exec "${NODE}" _robot:="${ROBOT}"
