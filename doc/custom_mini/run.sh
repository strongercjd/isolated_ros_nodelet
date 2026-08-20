#!/usr/bin/env bash
# =============================================================================
# custom_mini_runtime/run.sh
# -----------------------------------------------------------------------------
# 启动 ROS 环境：注入本目录 env，再启动 rosmaster。
# 不启动应用层 manager；应用层另开终端执行 app_runtime/run.sh。
#
# 本文件的源在 doc/custom_mini/。runtime 里是软链接。
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -L)"

if [[ ! -x "${HERE}/bin/rosmaster" ]]; then
  echo "runtime 不完整（缺少 bin/rosmaster），请先: ./custom_mini.sh package" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${HERE}/env.sh"

command -v python3 >/dev/null || { echo "需要系统 python3" >&2; exit 1; }

wait_tcp() {
  local host="$1" port="$2" i
  for i in $(seq 1 40); do
    if (echo >/dev/tcp/"${host}"/"${port}") 2>/dev/null; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

PIDS=()
cleanup() {
  echo
  echo "正在停止 ROS 环境 ..."
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

echo "==== custom_mini ROS 环境 ===="
echo "HERE=${HERE}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo

"${HERE}/bin/rosmaster" --core -p 11311 &
PIDS+=($!)
echo "等待 rosmaster ..."
if ! wait_tcp 127.0.0.1 11311; then
  echo "rosmaster 未能就绪" >&2
  exit 1
fi
echo "rosmaster 已就绪。"
echo "应用层请另开终端: ./app_runtime/run.sh"
echo "Ctrl+C 结束本 ROS 环境。"
wait
