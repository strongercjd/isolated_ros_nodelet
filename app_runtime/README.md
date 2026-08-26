# app_runtime — 应用层

```bash
./custom_mini.sh build && ./custom_mini.sh package

(cd app/heartbeat_nodelet && ./make.sh x86 && ./make.sh install)
(cd app/listener_nodelet && ./make.sh x86 && ./make.sh install)
(cd app/custom_ros_nodelet && ./make.sh x86 && ./make.sh install)
(cd app/slam2d_nodelet && ./make.sh x86 && ./make.sh install)   # Eigen 头文件已由 custom_mini.sh 内置提供

# 终端 1
./custom_mini.sh run

# 终端 2
./app_runtime/run.sh
```

| 路径 | 作用 |
|------|------|
| `bin/custom_ros_nodelet_manager` | 读 `plugins.json`，加载插件 |
| `lib/libheartbeat_nodelet.so` | 心跳（`app/heartbeat_nodelet`） |
| `lib/liblistener_nodelet.so` | 接收端（`app/listener_nodelet`） |
| `lib/libslam2d_nodelet.so` | 2D 激光 SLAM（`app/slam2d_nodelet`，详见其 README） |
| `plugins.json` | 插件列表 + 日志配置（见下） |
| `run.sh` | 检测 ROS Master 后启动 manager |
| `data/log/custom_ros_nodelet.log` | manager 与全部 nodelet 的运行日志 |

`/slam2d` 插件输入 remap 到 flat_sim 话题（`/mycar/base_scan`、`/mycar/odom`），
与 `./tool/flat_sim/run_flat_sim.sh` 联调：仿真器左半开车、右半实时看建图。

## 日志配置（plugins.json 顶层 "log" 字段）

manager 启动时读取，把自身与全部 nodelet 动态库的日志（rosconsole 的
stdout/stderr、printf 等）重定向进日志文件，时间戳为本地时区"月日时分秒"：

```json
"log": {
  "enabled": true,                                                    // false 时保持打印到终端
  "dir": "log",                                                       // 目录（相对 plugins.json 所在目录，可绝对路径）
  "file": "custom_ros_nodelet.log",                                   // 文件名（追加模式）
  "format": "[${severity}] [${time:%m-%d %H:%M:%S}] [${logger}] [${file}:${line}]: ${message}",  // ROSCONSOLE_FORMAT（${logger}=节点名，${file}:${line}=来源；${time:...} 为 boost time_facet 格式）
  "color": false                                                      // 文件日志默认关 ANSI 颜色
}
```

实时查看：`tail -f app_runtime/data/log/custom_ros_nodelet.log`。

实现要点：rosconsole 的格式/颜色在库静态初始化阶段即被定格，进程内补设
环境变量无效——manager 首次启动先 `setenv` 再 `execv` 自我重启（终端可见
`re-exec` 提示），重定向由 `dup2(stdout/stderr)` 完成，因此各级别日志均入文件。
