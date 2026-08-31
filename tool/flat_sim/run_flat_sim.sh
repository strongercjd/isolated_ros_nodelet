#!/usr/bin/env bash
# =============================================================================
# tool/flat_sim/run_flat_sim.sh —— 在已就绪的隔离 ROS 环境里跑 flat_sim
# -----------------------------------------------------------------------------
# 前置：
#   ./custom_mini.sh build           # 隔离 ROS → custom_mini_install/
#   ./custom_mini.sh run             # 另开终端：启动 ROS 环境（rosmaster）
#   ./tool/flat_sim/build.sh deps    # apt 装系统依赖（首次；headless 无需 GUI 库）
#   ./tool/flat_sim/build.sh         # 编译 flat_sim_node → tool/flat_sim/install/
#
# 本脚本只启动 flat_sim_node，不启动 rosmaster，也不启动 app_runtime。
# ROS 环境未就绪时退出，提示先跑 ./custom_mini.sh run。
#
# 流程：
#   1. source 隔离 ROS 环境（custom_mini_install）
#   2. 检测 rosmaster 是否可达；不可达则提示并退出
#   3. 运行 flat_sim_node 加载 box_house.fworld（默认 2D GUI；--headless 无窗口）
#      不发 TF、不发 /robot_description —— 坐标变换由应用层自行处理
#   4. 验证话题：/mycar/odom、/mycar/base_scan
#
# 用法：
#   ./run_flat_sim.sh              默认弹出 2D GUI 窗口
#   ./run_flat_sim.sh --headless   无界面（适合 CI / 无显示器环境）
#   ./run_flat_sim.sh --gui        显式 GUI（等同默认）
#   ./run_flat_sim.sh -h|--help    参数说明
#
# SLAM 建图视图已拆分至独立查看工具：tool/flat_sim_viewer/run_viewer.sh
# =============================================================================
set -uo pipefail

TOOL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../tool/flat_sim
REPO="$(dirname "$(dirname "${TOOL}")")"               # 仓库根（tool/flat_sim 上溯两级）
ROS_INSTALL="${REPO}/custom_mini_install"
FS_INSTALL="${TOOL}/install"
ROBOT_NAME="mycar"
HEADLESS=0

usage() {
  cat <<USAGE
用法: $0 [--gui | --headless | -h | --help]

  （无参数）   默认弹出 2D GUI 仿真窗口
  --gui        显式启动 GUI（等同默认）
  --headless   无界面运行（适合 CI / 无显示器环境）
  -h, --help   显示本说明

前置：请先在另一终端执行 ./custom_mini.sh run（启动 ROS 环境）。
本脚本不启动 rosmaster / app_runtime。
SLAM 建图视图：另见 tool/flat_sim_viewer/run_viewer.sh（独立查看工具）
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) HEADLESS=1; shift ;;
    --gui)      HEADLESS=0; shift ;;
    -h|--help)  usage; exit 0 ;;
    *) echo "未知参数：$1（可用 -h 查看帮助）" >&2; usage >&2; exit 1 ;;
  esac
done

# 世界文件优先用安装后的副本，找不到则退回源码目录。
WORLD="${FS_INSTALL}/share/flat_sim/worlds/box_house.fworld"
[[ -f "${WORLD}" ]] || WORLD="${TOOL}/worlds/box_house.fworld"

NODE="${FS_INSTALL}/bin/flat_sim_node"

[[ -d "${ROS_INSTALL}" ]] || { echo "缺少隔离 ROS（${ROS_INSTALL}），请先 ./custom_mini.sh build" >&2; exit 1; }
[[ -x "${NODE}" ]] || { echo "缺少 ${NODE}，请先 ./tool/flat_sim/build.sh" >&2; exit 1; }
[[ -f "${WORLD}" ]] || { echo "缺少世界文件 ${WORLD}" >&2; exit 1; }

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
export LD_LIBRARY_PATH="${FS_INSTALL}/lib:${LD_LIBRARY_PATH:-}"

# 隔离 ROS 的 setup.bash 只把 lib/python3/dist-packages 加入 PYTHONPATH，
# 但构建时装到 lib/python3.<ver>/site-packages 的纯 python 依赖不在这条路径上，
# 这里补齐（rospy 验证脚本需要）。
PY_SITE="${ROS_INSTALL}/lib/python$(python3 -c 'import sys;print("%d.%d"%sys.version_info[:2])')/site-packages"
export PYTHONPATH="${PY_SITE}:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1

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

echo "==== flat_sim demo ===="
echo "ROOT=${REPO}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "NODE=${NODE}"
echo "WORLD=${WORLD}"
if [[ "${HEADLESS}" -eq 1 ]]; then
  echo "模式: headless（无 GUI）"
else
  echo "模式: GUI（2D 俯视窗口；ESC 退出 / l 开关激光 / r 复位）"
fi
echo

if ! master_up "${MASTER_HOST}" "${MASTER_PORT}"; then
  echo "ROS 环境未启动（${MASTER_HOST}:${MASTER_PORT} 不可达）。" >&2
  echo "请先在另一终端执行: ./custom_mini.sh run" >&2
  echo "（或 ./custom_mini_runtime/run.sh）" >&2
  echo "不启动 flat_sim_node。" >&2
  exit 1
fi
echo "检测到 ROS Master 已就绪。"
echo

PIDS=()
cleanup() {
  echo
  echo "正在停止 flat_sim ..."
  trap - INT TERM EXIT
  for pid in "${PIDS[@]:-}"; do kill "${pid}" 2>/dev/null || true; done
  sleep 0.5
  for pid in "${PIDS[@]:-}"; do kill -9 "${pid}" 2>/dev/null || true; done
  echo "已停止。"
}
trap cleanup INT TERM EXIT

# flat_sim_node（GUI 模式关窗 / ESC 即退出；--headless 无窗口）
NODE_ARGS=(--world "${WORLD}")
if [[ "${HEADLESS}" -eq 1 ]]; then
  NODE_ARGS+=(--headless)
fi
echo "启动 flat_sim_node: ${NODE} ${NODE_ARGS[*]}"
"${NODE}" "${NODE_ARGS[@]}" &
PIDS+=($!)
sleep 3

# 验证话题（用 rospy：隔离环境没有 rosbag，Noetic 的 rostopic 在 main() 里
#    强制 import rosbag 会报错，故这里直接用 rospy 订阅验证）
echo
echo "==== 话题验证 ===="
python3 - "${ROBOT_NAME}" <<'PYCODE'
import sys, rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
name = sys.argv[1]
rospy.init_node("topic_check", anonymous=True)
print("已发布话题:")
for t, ty in rospy.get_published_topics():
    print("  %s  [%s]" % (t, ty))
ok = True
for topic, typ, label in (("/%s/odom" % name, Odometry, "odom"),
                          ("/%s/base_scan" % name, LaserScan, "base_scan")):
    try:
        rospy.wait_for_message(topic, typ, timeout=10)
        print("(/%s OK)" % label)
    except Exception as e:
        ok = False
        print("(未能收到 /%s: %s)" % (label, e))
sys.exit(0 if ok else 1)
PYCODE
rc=$?
echo
if [[ ${rc} -eq 0 ]]; then
  echo "==== 仿真运行正常，Ctrl+C 结束（不影响 ROS Master）===="
else
  echo "==== 话题验证未通过（rc=${rc}），Ctrl+C 结束 ====" >&2
fi
wait
