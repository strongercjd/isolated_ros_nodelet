#!/usr/bin/env bash
# =============================================================================
# tool/flat_sim_viewer/run_viewer.sh —— 在隔离 ROS 环境里跑 SLAM 建图查看器
# -----------------------------------------------------------------------------
# 前置：
#   ./custom_mini.sh build           # 隔离 ROS → custom_mini_install/
#   ./custom_mini.sh run             # 另开终端：启动 ROS 环境（rosmaster）
#   ./tool/flat_sim_viewer/build.sh deps    # apt 装系统依赖（首次）
#   ./tool/flat_sim_viewer/build.sh         # 编译 → tool/flat_sim_viewer/install/
#
# 本脚本只启动 flat_sim_viewer，不启动 rosmaster，也不启动 app_runtime。
# master 不可达时不退出：查看器本身会每秒重探并在状态栏显示连接状态。
#
# 流程：
#   1. source 隔离 ROS 环境（custom_mini_install）
#   2. 检测 rosmaster：可达则提示已就绪；不可达则提示（仍继续启动）
#   3. 启动 flat_sim_viewer（订阅 /slam2d/*：map / input_cloud / mapping_cloud / pose）
#
# 用法：
#   ./run_viewer.sh               启动查看器窗口
#   ./run_viewer.sh -h|--help     参数说明
#
# 仿真器（开车）另见：./tool/flat_sim/run_flat_sim.sh
# =============================================================================
set -uo pipefail

TOOL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../tool/flat_sim_viewer
REPO="$(dirname "$(dirname "${TOOL}")")"               # 仓库根
ROS_INSTALL="${REPO}/custom_mini_install"
VIEWER_INSTALL="${TOOL}/install"

usage() {
  cat <<USAGE
用法: $0 [-h | --help]

  （无参数）   启动 SLAM 建图查看器窗口（Qt6），默认实时订阅 /slam2d/*；
               窗口底部可切换"回放"并选择日志（app_runtime/data/log/ 下的
               map_log_*.bag，map_log_nodelet 录制）

窗口按键: v 恢复视图跟随  |  空格 播放/暂停  |  ←/→ 单步帧  |  ESC / q 退出
鼠标:     滚轮缩放（锚定光标）| 左键拖拽平移（脱离跟随）

前置：建议先在另一终端执行 ./custom_mini.sh run（启动 ROS 环境）。
master 未就绪时查看器以未连接状态启动，状态栏显示重试。
数据源：app/slam2d_nodelet 发布的 /slam2d/* 话题。
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1（可用 -h 查看帮助）" >&2; usage >&2; exit 1 ;;
  esac
done

NODE="${VIEWER_INSTALL}/bin/flat_sim_viewer"

[[ -d "${ROS_INSTALL}" ]] || { echo "缺少隔离 ROS（${ROS_INSTALL}），请先 ./custom_mini.sh build" >&2; exit 1; }
[[ -x "${NODE}" ]] || { echo "缺少 ${NODE}，请先 ./tool/flat_sim_viewer/build.sh" >&2; exit 1; }

# ---- 注入隔离环境 -----------------------------------------------------------
# ROS_* 必须先于 source 设置：custom_mini_install 的 profile 脚本（set -u 下）会读它们。
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

if [[ -f "${ROS_INSTALL}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROS_INSTALL}/setup.bash"
fi

# 本工具安装前缀（目前只有可执行文件；留 lib 路径兜底，便于以后拆动态库）。
export LD_LIBRARY_PATH="${VIEWER_INSTALL}/lib:${LD_LIBRARY_PATH:-}"

# rosbag 回放依赖 rospack 找到包（rosbag::Bag 构造即建 encryptor 的 pluginlib
# ClassLoader，ROS_PACKAGE_PATH 缺失会抛异常）；package.xml 在 install/share。
# 库解析另读 CMAKE_PREFIX_PATH（<prefix>/lib 下的加密插件库），一并挂上。
export ROS_PACKAGE_PATH="${ROS_INSTALL}/share${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
export CMAKE_PREFIX_PATH="${ROS_INSTALL}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"

# 从 ROS_MASTER_URI 解析 host:port（默认 127.0.0.1:11311）
parse_master() {
  local uri="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
  uri="${uri#http://}"
  uri="${uri#https://}"
  MASTER_HOST="${uri%%:*}"
  local rest="${uri#*:}"
  MASTER_PORT="${rest%%/*}"
  [[ -n "${MASTER_HOST}" ]] || MASTER_HOST=127.0.0.1
  [[ -n "${MASTER_PORT}" ]] || MASTER_PORT=11311
}

master_up() {
  local host="$1" port="$2"
  (echo >/dev/tcp/"${host}"/"${port}") 2>/dev/null
}

parse_master

echo "==== flat_sim_viewer ===="
echo "ROOT=${REPO}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "NODE=${NODE}"
echo

if master_up "${MASTER_HOST}" "${MASTER_PORT}"; then
  echo "检测到 ROS Master 已就绪。"
else
  # 查看器可以未连接启动（内部每秒重探），不视为错误
  echo "提示：ROS 环境未启动（${MASTER_HOST}:${MASTER_PORT} 不可达）。"
  echo "查看器将以未连接状态启动（状态栏显示重试）；master 就绪后自动订阅 /slam2d/*。"
fi
echo

PIDS=()
cleanup() {
  echo
  echo "正在停止 flat_sim_viewer ..."
  trap - INT TERM EXIT
  for pid in "${PIDS[@]:-}"; do kill "${pid}" 2>/dev/null || true; done
  sleep 0.5
  for pid in "${PIDS[@]:-}"; do kill -9 "${pid}" 2>/dev/null || true; done
  echo "已停止。"
}
trap cleanup INT TERM EXIT

echo "启动 flat_sim_viewer: ${NODE}"
"${NODE}" &
PIDS+=($!)
wait
