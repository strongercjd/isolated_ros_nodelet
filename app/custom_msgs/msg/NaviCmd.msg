# 导航控制命令（任务节点 -> base_navi，上层任务与基础导航解耦的通用协议）。
# 语义对齐 FASTBUILD::RunSearch/PauseSearch/ResumeSearch/StopSearch
uint8 CMD_START  = 0
uint8 CMD_PAUSE  = 1
uint8 CMD_RESUME = 2
uint8 CMD_STOP   = 3
uint8 cmd
uint32 seq
time stamp
