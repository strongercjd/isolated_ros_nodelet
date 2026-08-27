# tool/ —— 仓库工具集

`tool/` 存放与仓库 `src/` **相互独立**的辅助工具（仿真环境、调试工具等）。
每个工具一个**自包含子目录**，互不干扰：

```text
tool/
├── README.md          # 本文档（工具总览）
└── <工具>/            # 每个工具一个目录
    ├── README.md      # 该工具的说明
    ├── build.sh       # 独立编译脚本（deps / build / clean / help）
    ├── run_*.sh       # 运行脚本（按需）
    ├── src/           # 源码（缺失时由 build.sh 自动下载）
    ├── install/       # 编译产物（不写 custom_mini_install）
    └── build/         # 编译中间文件（build.sh 生成，已 gitignore）
```

## 工具列表

| 工具 | 说明 | 编译 | 运行 |
|------|------|------|------|
| [flat_sim](flat_sim/README.md) | 自研纯 2D 机器人仿真（.fworld/.frobot，无 TF / 无 URDF） | `./tool/flat_sim/build.sh` | `./tool/flat_sim/run_flat_sim.sh` |
| [flat_sim_teleop](flat_sim_teleop/README.md) | flat_sim 键盘遥控（方向键发 `/cmd_vel`） | `./tool/flat_sim_teleop/build.sh` | `./tool/flat_sim_teleop/run_teleop.sh` |

## 共同约定

- 都只依赖仓库根目录 `./custom_mini.sh build` 编出的**隔离 ROS 环境**（`custom_mini_install/`）。
- 工具**允许用 apt 安装**系统组件（如 SDL2），但必须在各自 README 里说明。
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
