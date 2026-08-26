#!/usr/bin/env bash
# =============================================================================
# official_full_runtime/run.sh
# -----------------------------------------------------------------------------
# 用本 runtime 自己的 bin / lib / python / share 跑官方路径的 Talker / Listener：
#   roscore → rosrun nodelet manager → load Talker / Listener
# 不依赖 official_full_install/。
#
# 本文件的源在 doc/official_full/。runtime 里是软链接。
# 经软链接调用时，HERE 必须解析成 runtime 目录，不能是 doc/official_full。
#   dirname + pwd -L：跟着逻辑路径走，不要用 pwd -P 追到 doc/。
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -L)"

# package 没跑完时这两个文件会缺，早点失败避免后面一堆 ModuleNotFoundError
if [[ ! -x "${HERE}/bin/roscore" || ! -x "${HERE}/lib/nodelet/nodelet" ]]; then
  echo "runtime 不完整（缺少 bin/roscore 或 lib/nodelet/nodelet），请先: ./official_full.sh package" >&2
  exit 1
fi

# catkin / pluginlib / rosrun 把 CMAKE_PREFIX_PATH 当成工作空间根（需要 .catkin）
export CMAKE_PREFIX_PATH="${HERE}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${HERE}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export ROS_PACKAGE_PATH="${HERE}/share${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
export PATH="${HERE}/bin:${PATH}"
export PYTHONPATH="${HERE}/python${PYTHONPATH:+:${PYTHONPATH}}"
export PKG_CONFIG_PATH="${HERE}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"

# 节点连本机 Master。未设置时给默认值，避免误连到别的 roscore。
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_DISTRO=noetic
export ROS_ROOT="${HERE}/share/ros"
export ROS_ETC_DIR="${HERE}/etc/ros"

# 不用用户目录里的 site-packages；日志立刻刷出
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1

# ROS 日志写到 runtime/.ros，不污染 $HOME/.ros
export ROS_HOME="${HERE}/.ros"
mkdir -p "${ROS_HOME}/log"

command -v python3 >/dev/null || { echo "需要系统 python3" >&2; exit 1; }

# 用本目录的 rosnode list 等某个节点名出现。
# 最多约 40 秒。失败多半是 roscore / manager 自己报错过，stderr 被丢掉以免刷屏。
wait_for_node() {
  local needle="$1"
  local i
  for i in $(seq 1 40); do
    if "${HERE}/bin/rosnode" list 2>/dev/null | grep -q "${needle}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

# 后台进程 PID。Ctrl+C / 脚本退出时一并杀掉（含 roscore 拉起的 master / rosout）。
PIDS=()
cleanup() {
  echo
  echo "正在停止 demo ..."
  trap - INT TERM EXIT
  local pid
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
  echo "已停止。"
}
trap cleanup INT TERM EXIT

echo "==== official_full_runtime ===="
echo "HERE=${HERE}"
echo "启动 roscore / nodelet manager / Talker / Listener ..."
echo "按 Ctrl+C 结束。"
echo

# roscore = roslaunch + rosmaster + /rosout。就绪标志是图上出现 /rosout。
"${HERE}/bin/roscore" &
PIDS+=($!)
echo "等待 ROS master ..."
if ! wait_for_node "/rosout"; then
  echo "roscore 未能在超时时间内就绪" >&2
  exit 1
fi
echo "ROS master 已就绪。"

# 官方用法：rosrun 按包名找到 lib/nodelet/nodelet，再 exec manager
"${HERE}/bin/rosrun" nodelet nodelet manager __name:=nodelet_manager &
PIDS+=($!)
if ! wait_for_node "/nodelet_manager"; then
  echo "nodelet manager 未能启动" >&2
  exit 1
fi
echo "nodelet manager 已就绪。"

# load 的类名来自各包 share/*/nodelet_plugins.xml
# 两个 load 都挂到同一个 manager；心跳 + Listener 在同一进程里
"${HERE}/bin/rosrun" nodelet nodelet load heartbeat_nodelet/HeartbeatNodelet nodelet_manager __name:=heartbeat &
PIDS+=($!)
sleep 1
"${HERE}/bin/rosrun" nodelet nodelet load listener_nodelet/ListenerNodelet nodelet_manager __name:=listener &
PIDS+=($!)

echo "Demo 已在前台运行。应周期性看到 Publishing / Received 日志。"
wait
