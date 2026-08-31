# 导航执行状态（base_navi -> 任务节点）。
# base_navi 只负责"到点"：目标选择与探索结束判断在任务层（fastbuild_task）。
# REACHED/ABORT 为驻留状态：零速待命，等任务层下发下一个 CMD_GOAL。
uint8 RUNNING  = 0   # 执行中（无目标时零速待命）
uint8 REACHED  = 1   # 到达当前目标，零速待命
uint8 ENDING   = 2   # 预留
uint8 PAUSED   = 3
uint8 ABORT    = 4   # 当前目标不可达（见 reason），零速待命
uint8 state

# ABORT 细分原因（其余状态填 REASON_NONE）
uint8 REASON_NONE      = 0
uint8 REASON_STUCK     = 1   # 反复顶死（3s 位移<1cm 连续 3 次）
uint8 REASON_GOAL_LOST = 2   # 持有目标超时未到达（"看得见够不着"防御）
uint8 reason

float32 pose_x          # map 系当前位姿
float32 pose_y
float32 pose_yaw        # rad
float32 target_x        # 当前目标；无目标时置 NaN
float32 target_y
time stamp
