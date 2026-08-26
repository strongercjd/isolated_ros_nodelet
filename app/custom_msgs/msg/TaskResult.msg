# 任务结果上报（job_node -> 外部观察）。
# 对齐原 task/TaskResult 话题 + WifiReportEvent 事件码
uint8 TASK_FASTBUILD = 13            # 对齐 JobId::FastBuildmap
uint32 EVT_FAST_BUILDING_FINISHED = 1127
uint32 EVT_FAST_BUILDING_FAILED  = 1115
uint8 task_id                       # = 13
uint8 result                        # 0=成功（对齐 ret=0）；非 0 对齐 finish_reason
uint32 event                        # 上报事件码（1127/1115）
float32 area_m2
float32 cost_time_s
string message                      # 人读摘要
time stamp
