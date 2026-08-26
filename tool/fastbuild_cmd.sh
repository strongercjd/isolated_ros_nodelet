#!/usr/bin/env bash
# =============================================================================
# tool/fastbuild_cmd.sh —— 终端里启动/取消快建任务、查看进度
# -----------------------------------------------------------------------------
# 用法:
#   ./tool/fastbuild_cmd.sh start     下发快建任务（JobCmd "fastbuild" → /job_node/cmd）
#   ./tool/fastbuild_cmd.sh cancel    取消进行中的快建任务
#   ./tool/fastbuild_cmd.sh status    监听任务/导航状态（Ctrl+C 退出）
#
# 前置（另开终端依次启动，见 app_runtime/README.md）:
#   ./custom_mini.sh run                        # 终端 1: ROS Master
#   ./tool/flat_sim/run_flat_sim.sh             # 终端 2: 仿真器（--headless 无窗口）
#   ./app_runtime/run.sh                        # 终端 3: nodelet manager（含 job/fastbuild 节点）
#
# 说明: 隔离 ROS 无 rosbag，Noetic 的 rostopic 无法使用；custom_msgs 也未注册为
#       ROS 包，故用 rospy + genpy 动态生成消息类发送。
# =============================================================================
set -euo pipefail

TOOL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "${TOOL}")"
ROS_INSTALL="${REPO}/custom_mini_install"
MSG_DIR="${REPO}/app/custom_msgs/msg"

usage() { sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

[[ $# -eq 1 ]] || { usage >&2; exit 1; }
case "$1" in
  start|cancel) CMD="$1" ;;
  status) CMD="$1" ;;
  -h|--help) usage; exit 0 ;;
  *) echo "未知命令: $1" >&2; usage >&2; exit 1 ;;
esac

[[ -d "${ROS_INSTALL}" ]] || { echo "缺少隔离 ROS: ${ROS_INSTALL}（先 ./custom_mini.sh build）" >&2; exit 1; }
(echo >/dev/tcp/127.0.0.1/11311) 2>/dev/null || { echo "ROS Master 未启动，请先: ./custom_mini.sh run" >&2; exit 1; }

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
PY_VER="$(python3 -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
export PYTHONPATH="${REPO}/python_compat:${ROS_INSTALL}/lib/python${PY_VER}/site-packages:${ROS_INSTALL}/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
export PYTHONNOUSERSITE=1

python3 -u - "${CMD}" "${MSG_DIR}" <<'PYCODE'
import os, sys, time
import rospy
from genpy.dynamic import generate_dynamic

cmd, msg_dir = sys.argv[1], sys.argv[2]

def load(name):
    text = open(os.path.join(msg_dir, name + '.msg')).read()
    return generate_dynamic('custom_msgs/' + name, text)['custom_msgs/' + name]

rospy.init_node('fastbuild_cmd', anonymous=True, disable_signals=False)

if cmd in ('start', 'cancel'):
    JobCmd = load('JobCmd')
    pub = rospy.Publisher('/job_node/cmd', JobCmd, queue_size=5)
    rospy.sleep(1.0)  # 等 TCP 连接建立，避免首发丢失
    m = JobCmd()
    m.seq = int(time.time()) & 0xffffffff
    m.command = 'fastbuild' if cmd == 'start' else 'cancel'
    m.stamp = rospy.Time.now()
    pub.publish(m)
    print('已发送 JobCmd(command="%s", seq=%d) → /job_node/cmd' % (m.command, m.seq))
    print('进度可看 manager 终端日志，或: ./tool/fastbuild_cmd.sh status')
else:  # status
    TaskState, NaviState = load('TaskState'), load('NaviState')
    TS = {1: 'RUNNING', 2: 'PAUSED', 3: 'RESUMED', 4: 'DONE'}
    RS = {0: 'IS_FINISH', 1: 'FAIL', 2: 'INTERRUPT', 3: 'ERROR', 255: '-'}
    NS = {0: 'RUNNING', 1: 'FINISH', 2: 'ENDING', 3: 'PAUSED', 4: 'ABORT'}
    def t_cb(m):
        if m.state == 4:
            print('[task] DONE reason=%s area=%.1f m2 cost=%.0f s' % (RS.get(m.finish_reason), m.area_m2, m.cost_time_s))
        else:
            print('[task] %s' % TS.get(m.state, m.state))
    def n_cb(m):
        tgt = 'none' if m.target_x != m.target_x else '(%.1f,%.1f)' % (m.target_x, m.target_y)
        print('[navi] %s pose=(%.2f,%.2f) tgt=%s frontier=%u ntc=%u area=%.1f m2' % (
            NS.get(m.state, m.state), m.pose_x, m.pose_y, tgt, m.frontier_count, m.no_target_count, m.explored_area_m2))
    rospy.Subscriber('/fastbuild_task/state', TaskState, t_cb)
    rospy.Subscriber('/base_navi/state', NaviState, n_cb)
    print('监听中（task 消息仅状态变化时发布；navi 1Hz 心跳）。Ctrl+C 退出。')
    rospy.spin()
PYCODE
