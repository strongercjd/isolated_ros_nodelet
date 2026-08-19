# custom_mini_runtime

不依赖 ROS `package.xml` / `nodelet_plugins.xml` 的最小可拷贝运行目录。插件列表在 `plugins.json`，由 `custom_mini_manager` 用 `class_loader` 按 `.so` 路径加载。

`doc/custom_mini/` 是 `README.md`、`run.sh`、`plugins.json` 的源文件；`custom_mini_runtime/` 里同名文件是指向这里的软链接。

```bash
./custom_mini.sh build     # 编译 → custom_mini_build/ + custom_mini_install/，日志 custom_mini_logs/
./custom_mini.sh package   # 抽出本目录（脚本与 JSON 为软链接）
./custom_mini.sh run       # 执行 custom_mini_runtime/run.sh
```

`run.sh` 只使用本目录，等价于：

```bash
bin/rosmaster --core -p 11311
bin/custom_mini_manager plugins.json
```

成功时终端应交替出现 `Publishing` / `Received`。Ctrl+C 结束。没有 `share/`。

## 目录

| 路径 | 作用 |
|------|------|
| `bin/` | 可执行文件：ROS Master + 本仓库的 manager |
| `lib/` | 插件 `.so` 以及 manager / 插件 `ldd` 扫到的非系统库 |
| `python/` | `rosmaster` 的 Python 运行环境（文件不再逐一说明） |
| `plugins.json` | 要加载的 Nodelet 列表 |
| `run.sh` | 启动 Master 与 manager |

下面「来源」是上游仓库；「本仓库」是对应的 `src/` 目录。编进 `official_full_install/` 或 `custom_mini_install/` 后，由 `custom_mini.sh package` 拷到这里。

## `bin/`

| 文件 | 作用 | 来源 | 本仓库 |
|------|------|------|--------|
| `custom_mini_manager` | 读 `plugins.json`，`dlopen` 插件并实例化 Talker / Listener | 本仓库编写，`./custom_mini.sh build` → `custom_mini_install/bin/` | `src/custom_mini` |
| `rosmaster` | ROS Master（话题/服务注册，Python） | [ros/ros_comm](https://github.com/ros/ros_comm)（`clients/rospy` / `tools/rosmaster`），`./official_full.sh build` | `src/ros_comm` |

## `lib/`

| 文件 | 作用 | 来源 | 本仓库 |
|------|------|------|--------|
| `libmy_nodelet_plugin.so` | Talker / Listener Nodelet | 本仓库编写 | `src/my_nodelet_plugin` |
| `libnodeletlib.so` | Nodelet 基类与进程内加载 | [ros/nodelet_core](https://github.com/ros/nodelet_core) | `src/nodelet_core` |
| `libclass_loader.so` | 按路径加载 `.so` 里的 C++ 类 | [ros/class_loader](https://github.com/ros/class_loader) | `src/class_loader` |
| `libbondcpp.so` | Nodelet 与 loader 之间的心跳 | [ros/bond_core](https://github.com/ros/bond_core) | `src/bond_core` |
| `libroscpp.so` | C++ ROS 客户端（发布/订阅） | [ros/ros_comm](https://github.com/ros/ros_comm) | `src/ros_comm` |
| `libxmlrpcpp.so` | 与 Master 通信的 XML-RPC | [ros/ros_comm](https://github.com/ros/ros_comm) | `src/ros_comm` |
| `librosconsole.so` / `librosconsole_print.so` / `librosconsole_backend_interface.so` | `ROS_INFO` 等日志 | [ros/rosconsole](https://github.com/ros/rosconsole) | `src/rosconsole` |
| `libroscpp_serialization.so` / `libcpp_common.so` / `librostime.so` | 消息序列化、公共类型、时间 | [ros/roscpp_core](https://github.com/ros/roscpp_core) | `src/roscpp_core` |
| `libroslib.so` | 包路径等辅助 | [ros/ros](https://github.com/ros/ros) | `src/ros` |
| `librospack.so` | 按包名查找（roscpp 链接依赖） | [ros/rospack](https://github.com/ros/rospack) | `src/rospack` |
| `libboost_thread.so.1.71.0` 等 | Boost 1.71（thread / filesystem / chrono / regex / program_options） | [Boost 1.71.0](https://www.boost.org/users/history/version_1_71_0.html)，脚本下 tar 编进 `official_full_install/` | `src/boost` |
| `libconsole_bridge.so.1.0` | ROS 与底层日志桥 | [ros/console_bridge](https://github.com/ros/console_bridge) | `src/console_bridge` |
| `libPocoFoundation.so.88` | `class_loader` 的 `dlopen` 后端 | [pocoproject/poco](https://github.com/pocoproject/poco) | `src/poco` |
| `libtinyxml2.so.8` | XML 解析（rospack / pluginlib） | [leethomason/tinyxml2](https://github.com/leethomason/tinyxml2) | `src/tinyxml2` |
| `libuuid.so.1` | UUID（bondcpp） | 本仓库编写 | `src/libuuid` |

除 `custom_mini_manager` 外，上表库均由 `./official_full.sh build` 编进 `official_full_install/lib/`，`package` 按 `ldd` 拷入。`libc` / `libstdc++` 等系统库不打包。

## `python/`

给 `bin/rosmaster` 用的隔离 Python 环境（`rosmaster`、`rosgraph`、`rospkg` 等）。`run.sh` 会把 `PYTHONPATH` 指到这里。具体模块不再展开。

## `plugins.json`

| 字段 | 说明 |
|------|------|
| `version` | 格式版本，当前为 `1` |
| `node` | `ros::init` 的节点名 |
| `defaults.library_dir` | 相对 JSON 所在目录的库目录（默认 `lib`） |
| `plugins[]` | 要加载的 Nodelet 列表，可随时加项 |
| `plugins[].name` | 实例名（如 `/talker`） |
| `plugins[].class` | `.so` 里注册的 C++ 类名（`PLUGINLIB_EXPORT_CLASS` 第一个参数） |
| `plugins[].library` | 动态库文件名或绝对路径 |
| `plugins[].enabled` | 可选，默认 `true` |
| `plugins[].remap` | 可选，话题重映射 `{from: to}` |
| `plugins[].args` | 可选，传给 nodelet 的参数数组 |
