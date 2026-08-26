# 快建任务命令（job -> fastbuild_task）。语义对齐 pb.task.Request.start_fastbuild_req
uint8 CMD_START  = 0   # 启动快建
uint8 CMD_CANCEL = 1   # 取消快建
uint8 cmd
uint32 seq                       # 递增序号，日志对账
string type                      # "build_map"（对齐 pb field 1；本环境仅实现 build_map）
bool start_from_charger          # 对齐 pb field 4（无充电桩，仅透传记录）
int32 map_type                   # 对齐 pb field 3
custom_msgs/FastBuildPoint[] expand_point   # 对齐 pb field 2（build_map 模式为空）
time stamp
