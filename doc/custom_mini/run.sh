#!/usr/bin/env bash
# =============================================================================
# custom_mini_runtime/run.sh
# -----------------------------------------------------------------------------
# 用本 runtime 自己的 bin / lib / python 跑 Talker / Listener。
# 不读 package.xml，不启动 roscore；只要 rosmaster + custom_mini_manager。
#
# 本文件的源在 doc/custom_mini/。runtime 里是软链接。
# 经软链接调用时，HERE 必须解析成 runtime 目录，不能是 doc/custom_mini。
#   dirname + pwd -L：跟着逻辑路径走，不要用 pwd -P 追到 doc/。
# =============================================================================
set -euo pipefail

# 经 runtime 下的软链接调用时，HERE 必须是 runtime 目录，不能是 doc/custom_mini
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -L)"

# 只使用本目录：动态库、rosmaster 的 Python、可执行文件
export LD_LIBRARY_PATH="${HERE}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${HERE}/python${PYTHONPATH:+:${PYTHONPATH}}"
export PATH="${HERE}/bin:${PATH}"

# 节点连本机 Master。未设置时给默认值，避免误连到别的 roscore。
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

# 不用用户目录里的 site-packages；日志立刻刷出，方便看 Publishing / Received
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1

# ROS 日志写到 runtime/.ros，不污染 $HOME/.ros
export ROS_HOME="${HERE}/.ros"
mkdir -p "${ROS_HOME}/log"

command -v python3 >/dev/null || { echo "需要系统 python3" >&2; exit 1; }

# 等 TCP 端口能连上。rosmaster 起来后会听 11311。
# bash 的 /dev/tcp 是内建，不依赖 nc。最多约 10 秒（40 × 0.25s）。
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

# 后台进程 PID。Ctrl+C / 脚本退出时一并杀掉，避免留下孤儿 rosmaster。
PIDS=()
cleanup() {
  echo
  echo "正在停止 ..."
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

echo "==== custom_mini_runtime ===="
echo "HERE=${HERE}"
echo "plugins=${HERE}/plugins.json"
echo

# --core：只当 Master，不另起 roslaunch；-p 11311 与 ROS_MASTER_URI 一致
"${HERE}/bin/rosmaster" --core -p 11311 &
PIDS+=($!)
echo "等待 rosmaster ..."
if ! wait_tcp 127.0.0.1 11311; then
  echo "rosmaster 未能就绪" >&2
  exit 1
fi
echo "rosmaster 已就绪。"

# manager 读 JSON，dlopen lib/ 里的插件，实例化 Talker / Listener
echo "启动 custom_mini_manager ..."
"${HERE}/bin/custom_mini_manager" "${HERE}/plugins.json" &
PIDS+=($!)

echo "运行中。应周期性看到 Publishing / Received。Ctrl+C 结束。"
wait
