# tool/ —— 仓库工具集

`tool/` 存放与仓库 `src/` **相互独立**的辅助工具（仿真环境、调试工具等）。
每个工具一个**自包含子目录**，互不干扰：

```text
tool/
├── README.md            # 本文档（工具总览）
├── set_custom_ros_env.sh  # 隔离 ROS 环境变量设置脚本（见下文）
└── <工具>/              # 每个工具一个目录
    ├── README.md        # 该工具的说明
    ├── build.sh         # 独立编译脚本（deps / build / clean / help）
    ├── run_*.sh         # 运行脚本（按需）
    ├── src/             # 源码（缺失时由 build.sh 自动下载）
    ├── install/         # 编译产物（不写 custom_mini_install）
    └── build/           # 编译中间文件（build.sh 生成，已 gitignore）
```

## 工具列表

| 工具 | 说明 | 编译 | 运行 |
|------|------|------|------|
| [flat_sim](flat_sim/README.md) | 自研纯 2D 机器人仿真（.fworld/.frobot，无 TF / 无 URDF） | `./tool/flat_sim/build.sh` | `./tool/flat_sim/run_flat_sim.sh` |
| [flat_sim_teleop](flat_sim_teleop/README.md) | flat_sim 键盘遥控（方向键发 `/cmd_vel`） | `./tool/flat_sim_teleop/build.sh` | `./tool/flat_sim_teleop/run_teleop.sh` |
| [flat_sim_viewer](flat_sim_viewer/README.md) | SLAM 建图查看（Qt6，订阅 `/slam2d/*`） | `./tool/flat_sim_viewer/build.sh` | `./tool/flat_sim_viewer/run_viewer.sh` |

## 环境脚本：set_custom_ros_env.sh

`tool/set_custom_ros_env.sh` 不是工具子目录，而是直接放在 `tool/` 下的顶层脚本：
在当前 shell 中启用 `custom_mini_install/` 隔离 ROS 环境（roscore / catkin 工具等），
供手动运行、调试各工具时使用。

用法（**必须在仓库根目录执行**，脚本用 `$PWD` 定位 `custom_mini_install/`）：

```bash
source tool/set_custom_ros_env.sh
```

导出的环境变量：

| 变量 | 内容 |
|------|------|
| `PATH` | 前置 `custom_mini_install/bin` |
| `LD_LIBRARY_PATH` | `custom_mini_install/lib` |
| `PYTHONPATH` | `python_compat` 及 `custom_mini_install` 下的 `python3.12/site-packages`、`python3/dist-packages` |

注意：

- 必须 `source`（或 `.`）执行；直接 `./set_custom_ros_env.sh` 运行会被脚本检测到并拒绝。
- 环境变量只在当前终端生效，新开终端需重新 source。

## 共同约定

- 都只依赖仓库根目录 `./custom_mini.sh build` 编出的**隔离 ROS 环境**（`custom_mini_install/`）。
- 工具**允许用 apt 安装**系统组件（如 Qt6），但必须在各自 README 里说明。
- 编译 / 安装产物在 `tool/<工具>/build/`、`tool/<工具>/install/`，已加入 `.gitignore`。
- 新增工具：在 `tool/` 下新建一个自包含目录，把上面的工具列表加一行即可。

## 新增一个工具的建议模板

```text
tool/<工具名>/
├── README.md      # 用途、依赖、用法
├── build.sh       # deps / build / clean / help 子命令
├── src/           # 源码
├── install/       # 编译产物
└── build/         # 中间文件
```
