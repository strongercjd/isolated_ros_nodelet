# custom_mini_runtime — ROS 环境

由 `./custom_mini.sh` 编译/打包的隔离 ROS 运行环境。不含应用层 manager / 插件。

```bash
./custom_mini.sh build     # 只编 ROS 栈 → custom_mini_install/
./custom_mini.sh package   # 抽出本目录
./custom_mini.sh run       # 启动 rosmaster（ROS 环境）
```

应用层请另开终端执行 `./app_runtime/run.sh`（需先 `app/*/make.sh install`）。

## 目录

| 路径 | 作用 |
|------|------|
| `bin/` | `rosmaster` |
| `lib/` | ROS 相关动态库（按关键库 `ldd` 收集） |
| `python/` | `rosmaster` 的 Python 依赖 |
| `env.sh` | source 后注入 PATH / LD_LIBRARY_PATH 等 |
| `run.sh` | source `env.sh` 并启动 `rosmaster` |
