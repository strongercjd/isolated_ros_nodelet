#!/usr/bin/env bash
# =============================================================================
# app_runtime/run.sh
# -----------------------------------------------------------------------------
# 应用层：检测 custom_mini ROS 环境（rosmaster）是否已就绪；
# 未就绪则退出，不启动 custom_ros_nodelet_manager。
# ROS 环境由 ./custom_mini_runtime/run.sh（或 ./custom_mini.sh run）启动。
#
# 本文件的源在 doc/app/。runtime 里是软链接。
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -L)"
ROS_RUNTIME="$(cd "${HERE}/../custom_mini_runtime" && pwd -L)"

if [[ ! -x "${HERE}/bin/custom_ros_nodelet_manager" \
   || ! -f "${HERE}/lib/libtalker_nodelet.so" \
   || ! -f "${HERE}/lib/liblistener_nodelet.so" ]]; then
  echo "app_runtime 不完整，请先分别:" >&2
  echo "  (cd app/custom_ros_nodelet && ./make.sh x86 && ./make.sh install)" >&2
  echo "  (cd app/talker_nodelet && ./make.sh x86 && ./make.sh install)" >&2
  echo "  (cd app/listener_nodelet && ./make.sh x86 && ./make.sh install)" >&2
  exit 1
fi
if [[ ! -f "${ROS_RUNTIME}/env.sh" ]]; then
  echo "缺少 custom_mini_runtime，请先: ./custom_mini.sh package" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${ROS_RUNTIME}/env.sh"
export LD_LIBRARY_PATH="${HERE}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PATH="${HERE}/bin:${PATH}"

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

echo "==== app_runtime ===="
echo "HERE=${HERE}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}"
echo "plugins=${HERE}/plugins.json"
echo

if ! master_up "${MASTER_HOST}" "${MASTER_PORT}"; then
  echo "ROS 环境未启动（${MASTER_HOST}:${MASTER_PORT} 不可达）。" >&2
  echo "请先在另一终端执行: ./custom_mini.sh run" >&2
  echo "（或 ./custom_mini_runtime/run.sh）" >&2
  echo "不启动 custom_ros_nodelet_manager。" >&2
  exit 1
fi
echo "检测到 ROS Master 已就绪。"

PIDS=()
cleanup() {
  echo
  echo "正在停止应用 ..."
  trap - INT TERM EXIT
  local pid
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  sleep 0.5
  for pid in "${PIDS[@]:-}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
  echo "已停止。"
}
trap cleanup INT TERM EXIT

echo "启动 custom_ros_nodelet_manager ..."
"${HERE}/bin/custom_ros_nodelet_manager" "${HERE}/plugins.json" &
PIDS+=($!)

echo "运行中。应周期性看到 Publishing / Received。Ctrl+C 结束应用（不影响 ROS Master）。"
wait
