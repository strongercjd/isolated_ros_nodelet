# 快建任务状态（fastbuild_task -> job_node 等）。
# state 值对齐原 task/FastbuildTaskState 话题：1=启动 2=暂停 3=恢复 4=结束；
# finish_reason 与结果字段对齐 pb.task.StartReturnRsp{ret, area}
uint8 STATE_RUNNING = 1
uint8 STATE_PAUSED  = 2
uint8 STATE_RESUMED = 3
uint8 STATE_DONE    = 4    # 本消息同时即最终结果
uint8 REASON_IS_FINISH = 0   # 对齐 FASTBUILD_TASK_IS_FINISH
uint8 REASON_FAIL      = 1
uint8 REASON_INTERRUPT = 2   # 对齐 FASTBUILD_TASK_INTERRUPT（被取消）
uint8 REASON_ERROR     = 3
uint8 state
uint8 finish_reason          # 仅 state==4 有效，其余填 255
float32 area_m2              # 对齐 start_return_rsp.area（成功时）
float32 cost_time_s          # 对齐 GetAreaTime().second
uint32 seq
time stamp
