# 导航控制命令（任务节点 -> base_navi，上层任务与基础导航解耦的通用协议）。
# base_navi 是纯执行器："从当前点到目标点"；目标点由任务层通过 CMD_GOAL 下发。
uint8 CMD_START  = 0   # 使能：进入 RUNNING（无目标时零速待命）
uint8 CMD_PAUSE  = 1
uint8 CMD_RESUME = 2
uint8 CMD_STOP   = 3   # 停止并回到 IDLE
uint8 CMD_GOAL   = 4   # 下发/更新目标点（RUNNING/REACHED/ABORT 状态接受）
uint8 cmd
uint32 seq
time stamp
float32 goal_x          # CMD_GOAL 有效：目标点（map 系）
float32 goal_y
float32 tolerance       # 到达判定容差（m）；<=0 用执行器默认值
