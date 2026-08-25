# flat_sim_teleop —— flat_sim 键盘遥控

通过方向键向 flat_sim 发布 `geometry_msgs/Twist`，控制仿真中的差速小车。

| 按键 | 动作 |
|------|------|
| ↑ | 前进 |
| ↓ | 后退 |
| ← | 逆时针旋转 |
| → | 顺时针旋转 |
| q / Ctrl+C | 退出（发送零速） |

默认控制机器人 `mycar`，话题 `/mycar/cmd_vel`（与 [flat_sim](../flat_sim/README.md) 一致）。

## 速度配置

修改 `src/TeleopConfig.h` 中的宏后重新编译即可：

```cpp
#define FLAT_SIM_TELEOP_LINEAR_SPEED  0.2   // m/s
#define FLAT_SIM_TELEOP_ANGULAR_SPEED 0.2   // rad/s
```

## 目录结构

```text
tool/flat_sim_teleop/
├── README.md
├── build.sh           deps / build / clean / help
├── run_teleop.sh      启动键盘遥控
├── CMakeLists.txt
├── src/
│   ├── TeleopConfig.h 速度宏
│   └── main.cpp
├── build/             中间文件（gitignore）
└── install/           flat_sim_teleop 可执行文件（gitignore）
```

## 使用方法

```bash
# 1. 隔离 ROS（若尚未构建）
./custom_mini.sh build

# 2. 编译遥控工具
./tool/flat_sim_teleop/build.sh

# 3. 终端 A：启动 flat_sim
./tool/flat_sim/run_flat_sim.sh

# 4. 终端 B：键盘遥控（需在真实 TTY 中运行）
./tool/flat_sim_teleop/run_teleop.sh

# 控制其他机器人名
./tool/flat_sim_teleop/run_teleop.sh --robot mycar
```

## 依赖

- 隔离 ROS：`custom_mini_install/`（`roscpp`、`geometry_msgs`）
- 系统：`build-essential`、`cmake`（`./build.sh deps` 可安装）

不依赖 flat_sim 的编译产物；运行时 flat_sim 须已在同一 `rosmaster` 上运行。
