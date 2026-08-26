# 调度命令入口（人工/外部 -> job_node）。
# 对齐原 task/WorkManage 服务的"收到快建命令"语义
uint32 seq
string command     # "fastbuild" = 下发快建 | "cancel" = 取消快建
int32 param        # 预留扩展参数
time stamp
