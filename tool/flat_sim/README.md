# flat_sim —— 自研 2D 机器人仿真（纯平面 / 无 TF / 无 URDF）

flat_sim 是仓库自研的**轻量 2D 仿真工具**：加载自研格式的世界/机器人文件，
做差速运动 + 激光射线仿真，并对接 ROS 话题。

GUI 为 Qt6 单窗口 2D 俯视仿真视图（世界边界、障碍、机器人、激光射线），
支持 `--headless` 无窗口运行。**SLAM 建图视图已拆分至独立工具
`tool/flat_sim_viewer`**（订阅 `/slam2d/*` 实时渲染占用地图、点云与位姿箭头）。

- 世界文件：**自研 `.fworld`**（缩进块 + `#` 备注）
- 机器人：**自研 `.frobot`**（形状 + 传感器挂载；**无 URDF**）
- 坐标变换：**不发 TF**，由应用层自行换算
- 构建：源码全在本目录，纯 CMake

定位：**仿真显示与基础运动/传感模拟**，不是完整物理引擎。

## 目录结构

```text
tool/flat_sim/
├── README.md            本文件
├── build.sh             编译脚本：deps / build / clean / help
├── run_flat_sim.sh      运行脚本：仅 flat_sim_node（需已有 ROS Master）；--gui / --headless
├── CMakeLists.txt       顶层 CMake（由 build.sh 调用，一般不手敲 cmake）
├── src/
│   ├── core/            世界模型、差速运动、激光射线、碰撞（不依赖 ROS/GUI）
│   ├── format/          .fworld / .frobot 解析（# 备注在解析层丢弃）
│   ├── gui/             Qt6 2D 俯视图 SimView + 控制器 SimApp（可选，headless 构建不链 Qt）
│   └── ros/             话题桥接（odom / base_scan / cmd_vel / heartbeat / slam2d/reset，
│                        无 TF）+ SimRunner（固定步长单步与 ROS 延迟初始化）
├── models/              示例机器人模型（.frobot）
├── worlds/              示例世界（.fworld）
├── build/               编译中间目录 + build.log（build.sh 生成，已 gitignore）
└── install/             安装前缀（build.sh 生成，已 gitignore）
```

只依赖仓库根 `./custom_mini.sh build` 产出的隔离 ROS（`custom_mini_install/`），
产物只写本工具 `build/`、`install/`，不污染 `custom_mini_install/`。

## 系统依赖（apt）

| 包 | 用途 |
|----|------|
| `build-essential` | g++ / make |
| `cmake` | 构建系统 |
| `qt6-base-dev` | Qt6 Widgets GUI（仅 GUI 需要；**headless 不需要，也不需要显示器**） |
| `qt6-base-dev-tools` | moc 等（随 qt6-base-dev 装入） |
| `libboost{,-system,-thread,-chrono}-dev` | Boost（系统版本 ≥ 1.83；roscpp 其余部分走隔离 ROS） |

`./build.sh deps` 会检测缺失并用 sudo 安装。

## 使用方法

```bash
# 1. 编出隔离 ROS（仅第一次，约 15-30 分钟）
./custom_mini.sh build

# 2. 安装系统依赖（仅第一次，需要 sudo）
./tool/flat_sim/build.sh deps

# 3. 编译 flat_sim → tool/flat_sim/install/（日志：build/build.log）
./tool/flat_sim/build.sh          # 默认 = build

# 4. 终端 A：启动 ROS 环境（rosmaster；本脚本不负责拉起）
./custom_mini.sh run

# 5. 终端 B：只启动 flat_sim 仿真
./tool/flat_sim/run_flat_sim.sh              # 默认：弹出 2D GUI 窗口
./tool/flat_sim/run_flat_sim.sh --gui        # 显式 GUI（等同默认）
./tool/flat_sim/run_flat_sim.sh --headless   # 无窗口（CI / 无显示器）
```

`run_flat_sim.sh` **不**启动 `rosmaster`，也**不**启动 `app_runtime`。
它会：检测 ROS Master 已就绪 → source 隔离环境 → 运行 `flat_sim_node`
加载 `worlds/box_house.fworld` → 用 rospy 验证 `/mycar/odom`、`/mycar/base_scan`
→ `Ctrl+C` 只停仿真（不影响 ROS Master）。Master 未就绪时提示先跑
`./custom_mini.sh run` 并退出。不发 `/robot_description`、不启动 TF。

**不要用 sudo 跑 `build.sh`**（`deps` 装包除外）；若曾用 sudo 构建导致目录 root 属主：
`sudo chown -R "$USER:$USER" tool/flat_sim/build` 后重来。

## 话题（默认机器人名 mycar）

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `/mycar/odom` | `nav_msgs/Odometry` | 里程计位姿与速度（频率 = 1/步长） |
| 发布 | `/mycar/base_scan` | `sensor_msgs/LaserScan` | 2D 激光（第 2 个激光起 `base_scan_2`…） |
| 订阅 | `/mycar/cmd_vel` | `geometry_msgs/Twist` | `linear.x` 线速度 m/s、`angular.z` 角速度 rad/s |
| 订阅 | `/heartbeat` | `std_msgs/Empty` | 控制节点心跳（`heartbeat_nodelet` 每 100ms）；首次收到后启用看门狗 |
| 发布 | `/slam2d/reset` | `std_msgs/Empty` | GUI 按 `r` 复位机器人时联动复位 SLAM |

- **心跳看门狗**：首次收到 `/heartbeat` 后，若连续 **500ms** 未再收到，视为控制节点停止，
  强制所有机器人 0 速并忽略 `cmd_vel`，直到心跳恢复。未收到过心跳时不干预（纯遥操仍可用）。
- **无 TF**：`frame_id` / `child_frame_id` 只是字符串占位（`odom` / `base_link` / `<name>/laser`），
  flat_sim 不维护、不广播 TF 树。应用层需要坐标变换时，用「车体位姿 + 激光相对位姿」
  在自己的节点里做三角函数换算即可。
- 消息时间戳默认 wall time；`rosrun flat_sim flat_sim_node ~sim_time:=true`（或
  remap 参数）可用累计仿真时间。

## 自研数据格式

单位：长度 **米**、角度 **度**；坐标系：**+x 东、+y 北、yaw 绕 +z 逆时针为正**（REP-103 惯例），
世界原点在中心（GUI 按此渲染）。文件为缩进块文本：`块名:` 一行，字段 `key: value` 缩进一层，
列表写在一行 `[a, b, c]`。

**备注**：`#` 到行尾全部丢弃（整行备注、行尾备注都支持；双引号内的 `#` 保留）。
备注不进入运行时模型——**删掉全部备注后仿真结果不变**。

### 世界文件 `.fworld`

```text
world:
  name: box_house
  size: [15, 10]          # 世界宽 x 高（GUI 视图适配用，可选）
  resolution: 0.02        # 兼容字段（解析法求交，不参与计算）
  timestep_ms: 50         # 仿真步长（毫秒），50 → 20 Hz

wall:                     # 矩形障碍（wall 与 box 同构，仅语义不同）
  name: wall_north
  pose: [0, 5, 0]         # 中心 [x, y, yaw_deg]
  size: [15, 0.15]        # 长, 宽（米，局部系；yaw 旋转）

circle:                   # 圆形障碍
  name: pillar_1
  pose: [-1, 0, 0]        # yaw 忽略
  radius: 0.3

robot:                    # 内嵌机器人（也可用 robot_file 引用，见下）
  ...
```

### 机器人 `.frobot`（三种写法语义一致）

独立文件 `models/mycar.frobot`：

```text
robot:
  name: mycar             # 亦用作话题前缀 /mycar/...
  shape: circle           # 首期仅 circle
  radius: 0.1
  pose: [0, 0, 0]         # 缺省初始位姿（world 里可覆盖）
  drive: diff             # 差速驱动
  color: red              # 仅显示用
  laser:                  # 可写多个
    pose: [0, 0, 0]       # 相对车体（前方为 0°，与车头同向）
    range: [0.0, 10.0]    # [min, max] 米
    fov_deg: 360          # 视场角（度）；360 = 全向，angle_min/max = ±180°
    samples: 361          # 线数
```

世界内引用（路径相对 `.fworld` 所在目录，缩进 `pose` 覆盖缺省位姿）：

```text
robot_file: "../models/mycar.frobot"
  pose: [6, -4, 90]
```

或直接在 `.fworld` 里写 `robot:` 内嵌块。示例：`worlds/box_house.fworld`（内嵌写法）、
`worlds/box_house_ref.fworld`（引用写法）。

解析行为：未知字段**告警忽略**（不崩溃）；格式错误报 `文件:行号: 原因` 并以非 0 退出；
`drive` 首期仅实现 `diff`，其他值告警后按 diff 处理。

## flat_sim_node 用法

```text
flat_sim_node --world <文件.fworld> [--gui | --headless] [-h]
```

- `--gui`（默认）：打开 Qt6 2D 俯视窗口（默认约 880×660，随窗口大小自适应）。
  按键：`ESC`/`q` 退出、`l` 开关激光显示、`r` 复位机器人并同步发 `/slam2d/reset`；
  关窗即退出。SLAM 建图查看另见 `tool/flat_sim_viewer`。
- `--headless`：不创建窗口，仿真与话题照常（无显示器环境必选）。
- 世界文件缺失 / 格式错误 → 打印 `文件:行号` 并退出码非 0。
- 未检测到 rosmaster 时仅本地仿真运行并告警，master 出现后自动挂话题（无需重启）。

## 构建选项

`CMakeLists.txt` 的 `option(FLAT_SIM_ENABLE_GUI ...)`（默认 ON）：
找到 Qt6（≥ 6.4）才编 GUI；找不到自动退化为纯 headless（`--gui` 将报错提示）。
完全关闭：`cmake -DFLAT_SIM_ENABLE_GUI=OFF ...`（不链任何 Qt）。

## 常见问题

| 现象 | 处理 |
|------|------|
| `缺少隔离 ROS` | 先 `./custom_mini.sh build` |
| `缺少 .../flat_sim_node` | 先 `./tool/flat_sim/build.sh` |
| `GUI 初始化失败（无 DISPLAY）` | 用 `./run_flat_sim.sh --headless` |
| 想用 `rostopic` 看话题 | 隔离环境没有 rosbag，Noetic 的 rostopic 启动即报错；请用 rospy 订阅（`run_flat_sim.sh` 内置验证即是） |
| 想看 SLAM 建图过程 | 用独立查看工具 `./tool/flat_sim_viewer/run_viewer.sh`（黑屏只有网格 = SLAM 插件未跑） |
| `ROS 环境未启动（… 不可达）` | 先在另一终端执行 `./custom_mini.sh run`（或 `./custom_mini_runtime/run.sh`） |
| 机器人一开始就动不了 | 初始位姿可能与障碍重叠（启动时有告警）；改 world 里 robot 的 `pose` |
