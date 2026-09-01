# map_log_nodelet — SLAM 建图日志记录插件

订阅 `slam2d_nodelet` 发布的建图话题，写入 rosbag（LZ4 压缩）日志，
由机器人运动指令门控：**车动才记，车停即停**。日志供
`tool/flat_sim_viewer` 回放（数据源切换 → 打开 Bag…）。

## 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/slam2d/map` | `nav_msgs/OccupancyGrid` | 订阅 | 占用栅格地图（1Hz latch） |
| `/slam2d/pose` | `geometry_msgs/PoseStamped` | 订阅 | 机器人位姿 |
| `/slam2d/input_cloud` | `sensor_msgs/PointCloud2` | 订阅 | 配准前点云 |
| `/slam2d/mapping_cloud` | `sensor_msgs/PointCloud2` | 订阅 | 配准后点云 |
| `/mycar/cmd_vel` | `geometry_msgs/Twist` | 订阅 | **门控**：`linear.x != 0 \|\| angular.z != 0` 即记录（精确比较、无阈值；保持式 cmd_vel 静止时持续发零值） |

## 日志文件

- 位置：`app_runtime/data/log/`（目录不存在自动创建）
- 命名：`map_log_<YYYYmmdd_HHMMSS>_segNNN.bag`，时间戳为插件启动时刻，
  `NNN` 段号在**每次门控重新打开时递增**——一次连续运动一段文件，不续写旧段
- 门控打开瞬间补写四类话题的最近值（同一时间戳），回放第一帧即有完整画面
- bag 记录时间戳 = 写入时刻（非消息 `header.stamp`），保证时间轴严格单调
- 段文件在 `kClose`（静止）或进程正常退出时 close 落索引；`kill -9`
  会丢当前段索引，可用 `custom_mini_install/bin/rosbag reindex <file>` 修复

## 日志目录解析（优先级）

1. `~log_dir` 参数显式指定
2. `dladdr` 从本 .so 加载路径推导 `<app_runtime>/data/log`（默认路径，不受启动 CWD 影响）
3. `/proc/self/exe` 推导（dladdr 失败的回退）
4. CWD 下 `map_log/`（最后兜底，会打 WARN）

## 参数（私有句柄 `~`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `cmd_vel_topic` | `/mycar/cmd_vel` | 门控话题 |
| `log_dir` | 自动推导 | 日志目录（绝对路径） |
| `queue_max` | `64` | 落盘队列深度，满时丢最旧数据项 |

## 构建与注册

```bash
cd app/map_log_nodelet && ./make.sh x86 && ./make.sh install
```

`app_runtime/plugins.json` 的 `plugins` 数组需含：

```json
{ "name": "/map_log", "class": "map_log_nodelet::MapLogNodelet",
  "library": "libmap_log_nodelet.so", "enabled": true }
```

## 线程模型

manager 是单线程 spin：回调只做引用计数浅拷贝 + 入队；序列化与写盘在
独立 worker 线程（仿 `slam2d_nodelet` 的有界队列模式）。`rosbag::Bag`
仅 worker 线程触碰。
