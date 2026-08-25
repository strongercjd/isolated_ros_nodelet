# slam2d_nodelet — 2D 激光 SLAM（app 层插件）

移植自教学版 2D 激光 SLAM 参考工程（`lidarslam_2d-master/`，仓库外参考代码）：
odom 预测 + 点云粗匹配（scan-to-scan）+ ICP 精配准（scan-to-map）+ raycast 栅格建图。
算法仅依赖 **Eigen**（无 PCL / Ceres / g2o / tf），以 nodelet 插件形式运行在
`app_runtime/bin/custom_ros_nodelet` manager 上。

## 算法结构（`src/slam2d/`，自参考工程 vendoring）

| 模块 | 文件 | 作用 |
|------|------|------|
| 前端流程 | `slam/front_end_process.*` | 每帧：odom 增量预测 → 粗匹配 → scan-to-map ICP → 建图 |
| 粗匹配 | `slam/laser_odometry.*` | CSM 式相关性扫描匹配（平移×旋转候选打分） |
| 精配准 | `slam/icp2d.*` + `slam/point_to_point.*` | 点到点 ICP（SVD 解析，`max_match_distance=0.05`） |
| 建图 | `slam/laser_mapping.*` | raycast 更新栅格（hit -8 / free +4，值域 [-127,127]） |
| 基础库 | `unity/{transform,grid_map,voxel_filter,point_cloud,tic_toc}` | 刚体变换、栅格地图、体素滤波 |

首帧（`frame_id_ < 1`）直通建图；`< 10` 帧跳过 ICP；之后每帧做 scan-to-map 配准。
地图 2000×2000、分辨率 0.05 m（100 m × 100 m，原点居中）。

## 与参考工程的差异（算法逻辑零改动，仅接口适配）

- **去 UI / rosbag / tf / Ceres**：cv_ui 画布、rosbag_reader、tf 监听全部剥离；
  Ceres 只剩注释（原工程就没实际用）。
- **`unity/transform.cpp` 修复了 `Rigid2f::inverse()` 的 bug**（参考工程原代码
  误用 `new_pos(-(_pose))`，平移没有随旋转一起求逆，仅在角度为 0 时碰巧正确；
  初始朝向非 0 时 odom 增量会整体转错方向——实测位姿从 (6,-4,90°) 瞬间跳到
  (8,-14)）。正确公式 `-(R⁻¹·pose)` 见该文件注释。
- **`slam/laser_odometry.cpp` 修复了 CSM 粗匹配的发散缺陷**：原实现以零位姿为
  基准全范围搜索（±30°、步长 5°）且不用 odom 初值（`Process` 的 odom 参数
  只在被注释的代码里出现过；`base_angle` 参数预留了但从未传入，且传入时
  候选角还会丢基准角）——评分又只对"匹配上的点"取均值，贴墙原地旋转时
  错位候选（少数点碰巧贴墙、其余点被丢弃）反而得分更高，实测 90° 原地
  转向累计跟丢 ~74°、位置发散 3.9 m。修复：CSM 以 odom 帧间变换为搜索中心
  （±10°、步长 2°，平移同加基准），`initial_delta` 以 odom 兜底；修复后
  贴墙原地转偏差 ≤0.06 m，绕圈全程（~60 m）最大偏差 0.15 m。
- **`slam/point_to_point.cpp` 显式 `#include <Eigen/SVD>` / `<Eigen/LU>`**：
  原由 `<ceres/ceres.h>` 间接引入，删 Ceres 后缺了会懒加载崩溃（determinant
  的 2×2 特化在 LU 模块）。
- **构造注入初始位姿**：`FrontEndProcess(init_pose)`。数据源（flat_sim）的 odom
  是世界系绝对位姿，首帧 odom 位姿作 SLAM 初值后**地图系与世界系对齐**，
  仿真视图与建图视图可直接视觉对照。
- **nodelet 化 + worker 线程**：manager 是 `ros::spin()` 单线程，算法不能占回调。
  回调只做 odom/scan 配对与格式转换（µs 级）入队，独立 worker 线程跑算法并发布；
  队列上限 3，算不过来丢最旧取最新（20 Hz 下降级不积压）。worker 循环整体
  try/catch，异常不会连带 manager 内其他插件退出。

## 话题

输入名走 remap（`app_runtime/plugins.json`）：

| 方向 | 话题（remap 后实际名） | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `scan` → `/mycar/base_scan` | `sensor_msgs/LaserScan` | 2D 激光 |
| 订阅 | `odom` → `/mycar/odom` | `nav_msgs/Odometry` | 里程计（世界系绝对位姿） |
| 订阅 | `reset` → `/slam2d/reset` | `std_msgs/Empty` | 复位：重建前端，地图重新对齐 |
| 发布 | `/slam2d/map` | `nav_msgs/OccupancyGrid` | 1 Hz 节流 + latch；occ 0-100（50=未观测） |
| 发布 | `/slam2d/pose` | `geometry_msgs/PoseStamped` | 配准后位姿（map 系） |
| 发布 | `/slam2d/input_cloud` | `sensor_msgs/PointCloud2` | 配准前点云（红，xy 两字段） |
| 发布 | `/slam2d/mapping_cloud` | `sensor_msgs/PointCloud2` | 配准后点云（蓝，xy 两字段） |

OccupancyGrid 发布时做了行翻转（内部 GridMap row0 在 y 最大侧，标准消息 row0
在 y 最小侧）与值域映射（+127 自由→0，-127 占用→100，0 未观测→50）。

## 参数（私有句柄 `~`，plugins.json `args` 或默认值）

| 参数 | 默认 | 说明 |
|------|------|------|
| `~scan_decimate` | `1` | 每 N 帧取 1 帧处理（算力不足时调大） |
| `~map_publish_period` | `1.0` | 地图发布周期（秒） |
| `~publish_clouds` | `true` | 是否发布红/蓝点云（调试用，可关省带宽） |

## 构建与运行

```bash
# 依赖：隔离 ROS 需含 Eigen（custom_mini.sh 已内置 setup_eigen）
(cd app/slam2d_nodelet && ./make.sh x86 && ./make.sh install)
# 产物：app_runtime/lib/libslam2d_nodelet.so

# 终端 1：rosmaster
./custom_mini.sh run
# 终端 2：仿真器（数据源 + 分屏显示）
./tool/flat_sim/run_flat_sim.sh
# 终端 3：manager 加载插件（plugins.json 已含 /slam2d 条目）
./app_runtime/run.sh
```

`plugins.json` 条目（已配置，此处备查）：

```json
{
  "name": "/slam2d",
  "class": "slam2d_nodelet::Slam2dNodelet",
  "library": "libslam2d_nodelet.so",
  "enabled": true,
  "remap": {
    "/slam2d/scan": "/mycar/base_scan",
    "/slam2d/odom": "/mycar/odom",
    "/slam2d/reset": "/slam2d/reset"
  },
  "args": []
}
```

验证：manager 日志出现 `loaded /slam2d`；处理帧日志
`frame #N pose(x, y, yaw deg) cost M ms`（box_house 场景实测 ~33 ms/帧，
低于 20 Hz 周期 50 ms）。遥控开车后 flat_sim 右半视图可见墙体闭合的灰度地图。

## 目录

```text
app/slam2d_nodelet/
├── CMakeLists.txt          照 talker_nodelet 模式（SHARED + PIC）
├── make.sh                 x86 编译 / install 拷 .so → app_runtime/lib
├── include/slam2d_nodelet.h
└── src/
    ├── slam2d_nodelet.cpp  nodelet：订阅/队列/worker/发布
    ├── slam_input.h        odom-scan 同戳配对 + scan → 点云（header-only）
    └── slam2d/             参考工程算法源码（unity/ + slam/）
```
