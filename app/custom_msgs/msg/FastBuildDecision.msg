# 快建任务单次选点决策（fastbuild_task 决策层发布；供 flat_sim_viewer 在线显示与日志回放）
# 语义：一次决策 = 当前地图上的候选点位 + 本轮最终选中的目标点（或未选出）+ 拉黑点。
# 日志回放侧按 header.stamp（ros::Time）与 map_log bag 的时间轴匹配。
uint32 seq                 # 决策序号（递增）
time stamp                 # 决策时刻（ros::Time）
custom_msgs/FastBuildPoint[] candidates   # 候选点位（世界系 x,y；theta 恒 0）
uint32[] candidate_size    # 各候选簇大小（与 candidates 对齐；画点大小/排序用）
uint8[]  candidate_via     # 0=直线可达 1=绕行（与 candidates 对齐；选点罚分结果）
bool has_selected          # 本轮是否选出目标
float32 selected_x         # 最终选中的目标点（has_selected 为真时有效）
float32 selected_y
custom_msgs/FastBuildPoint[] blacklist    # 当前拉黑点（1m 邻域内不再选）
float32 area_m2            # 最近一张地图的已探索面积
uint8 task_state           # 1=RUNNING 4=DONE（对齐 custom_msgs/TaskState）
