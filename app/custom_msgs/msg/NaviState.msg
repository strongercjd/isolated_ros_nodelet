# 导航执行状态（base_navi -> 任务节点）。
# state 值对齐 FASTBUILD::EXPLORESTATE：RUNNING=0/FINISH=1/ENDING=2/PAUSED=3/ABORT=4
uint8 RUNNING      = 0
uint8 FINISH       = 1    # 连续无可达目标，自然结束
uint8 ENDING       = 2    # 收尾中（预留）
uint8 PAUSED       = 3
uint8 ABORT        = 4    # 内部错误/超时
uint8 state

float32 pose_x                # map 系当前位姿（=flat_sim 世界系）
float32 pose_y
float32 pose_yaw              # rad
float32 target_x              # 当前 frontier 目标；无目标时置 NaN
float32 target_y
uint32 frontier_count         # 本轮候选 frontier 聚类数
uint32 no_target_count        # 连续无可达目标轮数（对齐 explorestatestopcount）
float32 explored_area_m2      # 已探索自由面积（对齐 GetAreaTime 的 area）
time stamp
