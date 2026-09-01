# flat_sim_viewer —— SLAM 建图查看工具（Qt6）

flat_sim_viewer 是从 flat_sim 拆分出的**独立可视化工具**：订阅
`app/slam2d_nodelet` 发布的 `/slam2d/*` 话题，实时渲染 SLAM 建图过程。
查看器**必有 UI**（无 headless 形态），无 rosmaster 时以未连接状态启动，
状态栏提示并在后台每秒重探。

显示内容与参考工程 lidarslam_2d 的画布一致：

- 灰度占用栅格地图（自由白 / 占用黑 / 未观测灰）
- 5 m 网格线
- 红色 `input_cloud`（配准前点云）
- 蓝色 `mapping_cloud`（配准后点云）
- 蓝色位姿箭头

默认视图跟随最新位姿（机器人初始不在原点时箭头也不会跑出视口），
拖拽平移即脱离跟随，`v` 键恢复。

## 目录结构

```text
tool/flat_sim_viewer/
├── README.md            本文件
├── build.sh             编译脚本：deps / build / clean / help
├── run_viewer.sh        运行脚本（支持 --bag 直启回放）
├── CMakeLists.txt       顶层 CMake（由 build.sh 调用，一般不手敲 cmake）
├── src/
│   ├── app/             ViewerWindow：主窗口 + 播放条 + 状态栏 + 定时器数据泵
│   ├── view/            SlamView：Qt6 自绘画布（视图数学自 flat_sim SDL 版迁移）
│   ├── replay/          BagPlayer：rosbag 回放引擎（无 Qt，轮询式）
│   └── ros/             SlamListener（实时源）/ SlamSnapshot / SnapshotBuilder
├── tests/               test_bagplayer：BagPlayer headless 回归测试（无 Qt）
├── build/               编译中间目录 + build.log（build.sh 生成，已 gitignore）
└── install/             安装前缀（build.sh 生成，已 gitignore）
```

只依赖仓库根 `./custom_mini.sh build` 产出的隔离 ROS（`custom_mini_install/`）
与系统 Qt6 / Boost，产物只写本工具 `build/`、`install/`。

## 系统依赖（apt）

| 包 | 用途 |
|----|------|
| `build-essential` | g++ / make |
| `cmake` | 构建系统 |
| `qt6-base-dev` | Qt6 Widgets GUI（**必需**，查看器必有 UI） |
| `qt6-base-dev-tools` | moc 等（随 qt6-base-dev 装入） |
| `libboost{,-system,-thread,-chrono}-dev` | Boost（系统版本 ≥ 1.83） |

`./build.sh deps` 会检测缺失并用 sudo 安装。

## 使用方法

```bash
# 1. 编出隔离 ROS（仅第一次）
./custom_mini.sh build

# 2. 安装系统依赖（仅第一次，需要 sudo）
./tool/flat_sim_viewer/build.sh deps

# 3. 编译 → tool/flat_sim_viewer/install/
./tool/flat_sim_viewer/build.sh

# 4. 启动环境（终端 A：rosmaster）
./custom_mini.sh run

# 5. 启动应用层（终端 B：含 slam2d 插件）
./app_runtime/run.sh

# 6. 启动仿真器（终端 C：数据源头，开车）
./tool/flat_sim/run_flat_sim.sh          # 或 --headless

# 7. 启动查看器（终端 D）
./tool/flat_sim_viewer/run_viewer.sh

# 回放本地日志（app/slam2d_nodelet 录制的建图过程，另见 app/map_log_nodelet）
./tool/flat_sim_viewer/run_viewer.sh --bag app_runtime/data/log/map_log_*.bag
```

## 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/slam2d/map` | `nav_msgs/OccupancyGrid` | 订阅 | 占用栅格地图（latch，1Hz 节流） |
| `/slam2d/input_cloud` | `sensor_msgs/PointCloud2` | 订阅 | 配准前点云（红） |
| `/slam2d/mapping_cloud` | `sensor_msgs/PointCloud2` | 订阅 | 配准后点云（蓝） |
| `/slam2d/pose` | `geometry_msgs/PoseStamped` | 订阅 | 机器人位姿（蓝箭头） |

查看器纯被动，不发布任何话题（`/slam2d/reset` 由 flat_sim 的 r 键联动发布）。

## 状态栏

| 标签 | 内容 |
|------|------|
| 连接 | `● 未连接 rosmaster（重试中）` / `● 已连接 · 等待 SLAM 数据` / `● 数据正常` / `● 回放：<文件名>` |
| 位姿 | `x=+0.09  y=+0.03  θ=+38.0°`（无数据时 `—` 占位） |
| 地图 | `400×400 @0.05m  seq=121`（宽×高 @分辨率 代数） |
| 视图 | `0.010 m/px · 跟随中` / `· 自由视图` |

## 数据源：实时 / 回放

窗口底部播放条左侧两个按钮切换数据源（渲染层共用，随时互切）：

- **实时**（默认）：订阅 `/slam2d/*`，与旧版行为一致；此时回放控件全部隐藏
- **回放**：自动弹出文件对话框（起始目录即 `app_runtime/data/log/`）选择日志
  （`map_log_nodelet` 录制的 `map_log_*.bag`），或启动时 `--bag <file>` 直达。
  **加载完成后自动开始播放**

回放为完整播放器：

| 控件/按键 | 效果 |
|------|------|
| `空格` / ▶ 按钮 | 播放 / 暂停（播到末尾自动暂停，再按从头重播） |
| `→` / `←`（或 \|◀ ▶\| 按钮） | 单步进 / 退一帧（帧 = 一次 pose，对齐一次视觉变化） |
| 进度条 | 拖动到任意时刻（松开生效，检查点重建，毫秒级） |
| 倍速下拉 | 0.5× / 1× / 2× |
| 右侧标签 | `mm:ss.s / mm:ss.s · 帧 k/N · 倍速` |

回放模式下实时订阅被挂起（不 shutdown ROS）；切回实时时 latch 的地图
即刻恢复显示。回放文件由运动门控分段（一段 = 一次连续运动），每段开头
都补写了完整的地图/位姿/双点云快照，第一帧即有完整画面。

## 交互

| 操作 | 效果 |
|------|------|
| 滚轮 | 缩放（锚定光标，0.001 ~ 0.2 m/px） |
| 左键拖拽 | 平移（即脱离跟随） |
| `v` | 恢复默认缩放与跟随 |
| `ESC` / `q` | 退出 |
| `Ctrl+C`（终端） | 退出（状态栏轮询 ros::ok()） |

## FAQ

**地图一直灰色只有网格？**
SLAM 未运行或仿真器没发数据。确认 `./app_runtime/run.sh`（含 slam2d 插件）
与 `./tool/flat_sim/run_flat_sim.sh` 都在跑；地图只在机器人运动建图后出现。

**状态栏显示"未连接"？**
`./custom_mini.sh run` 未启动或 ROS_MASTER_URI 不对；查看器会每秒自动重试，
master 就绪后自动订阅，无需重启。
