# official_full_runtime

官方路径的最小可拷贝运行目录：`roscore` + `rosrun nodelet` 加载 Talker / Listener。`doc/official_full/` 是 `README.md`、`run.sh` 的源文件；`official_full_runtime/` 里同名文件是软链接。

`./official_full.sh package` 从 `official_full_install/` 抽出运行所需文件，不拷整棵 install。`run.sh` 只使用本目录。

```bash
./official_full.sh build
./official_full.sh package
./official_full.sh run
```

成功时终端应交替出现 `Publishing` / `Received`。Ctrl+C 结束。

## 目录

| 路径 | 作用 |
|------|------|
| `bin/` | `roscore`、`rosrun`、`rosnode` 等官方入口 |
| `lib/` | `nodelet` / `rosout` 可执行文件、插件 `.so`，以及 `ldd` 扫到的非系统库 |
| `python/` | `roscore` / `rosmaster` / `rosnode` 的 Python 运行环境（文件不再逐一说明） |
| `share/` | pluginlib / `rosrun` 查找包用的 `package.xml`、`nodelet_plugins.xml` |
| `.catkin` | 空标记文件，让 `CMAKE_PREFIX_PATH` 被当成 catkin 工作空间 |
| `run.sh` | 启动 roscore → manager → load Talker / Listener |

下面「来源」均由 `./official_full.sh build` 编进 `official_full_install/`，再由 `package` 拷到这里。

## `bin/`

| 文件 | 作用 | 来源 |
|------|------|------|
| `roscore` | 拉起 Master + `/rosout` | [ros/ros_comm](https://github.com/ros/ros_comm)（`tools/roslaunch`） |
| `rosmaster` | ROS Master（Python） | [ros/ros_comm](https://github.com/ros/ros_comm) |
| `rosrun` | 按包名找并执行 `lib/<pkg>/<exe>`（bash） | [ros/ros](https://github.com/ros/ros)（`tools/rosbash`） |
| `rosnode` | 等 `/rosout`、`/nodelet_manager` 是否就绪 | [ros/ros_comm](https://github.com/ros/ros_comm) |
| `rosversion` | `roscore.xml` 写入 `/rosversion`、`/rosdistro` | pip 包 `rospkg`（装进 `official_full_install/`） |
| `rospack` | 按包名解析 `share/<pkg>` | [ros/rospack](https://github.com/ros/rospack) |
| `catkin_find` | `rosrun` 用来定位 `lib/<pkg>` | [ros/catkin](https://github.com/ros/catkin) |

## `lib/`

### 可执行文件

| 文件 | 作用 | 来源 |
|------|------|------|
| `lib/nodelet/nodelet` | 官方 nodelet 管理器 / `load` | [ros/nodelet_core](https://github.com/ros/nodelet_core) |
| `lib/rosout/rosout` | 系统日志节点（`roscore` 拉起） | [ros/ros_comm](https://github.com/ros/ros_comm) |

### 共享库

| 文件 | 作用 | 来源 |
|------|------|------|
| `libmy_nodelet_plugin.so` | Talker / Listener Nodelet | 本仓库 `src/my_nodelet_plugin` |
| `libnodeletlib.so` | Nodelet 基类与进程内加载 | [ros/nodelet_core](https://github.com/ros/nodelet_core) |
| `libclass_loader.so` | 按路径加载 `.so` 里的 C++ 类 | [ros/class_loader](https://github.com/ros/class_loader) |
| `libbondcpp.so` | Nodelet 与 loader 之间的心跳 | [ros/bond_core](https://github.com/ros/bond_core) |
| `libroscpp.so` | C++ ROS 客户端 | [ros/ros_comm](https://github.com/ros/ros_comm) |
| `libxmlrpcpp.so` | 与 Master 通信的 XML-RPC | [ros/ros_comm](https://github.com/ros/ros_comm) |
| `librosconsole.so` / `librosconsole_print.so` / `librosconsole_backend_interface.so` | `ROS_INFO` 等日志 | [ros/rosconsole](https://github.com/ros/rosconsole) |
| `libroscpp_serialization.so` / `libcpp_common.so` / `librostime.so` | 消息序列化、公共类型、时间 | [ros/roscpp_core](https://github.com/ros/roscpp_core) |
| `libroslib.so` | 包路径等辅助 | [ros/ros](https://github.com/ros/ros) |
| `librospack.so` | 按包名查找 | [ros/rospack](https://github.com/ros/rospack) |
| `libboost_*.so.1.71.0` | Boost 1.71 | [Boost 1.71.0](https://www.boost.org/users/history/version_1_71_0.html) |
| `libconsole_bridge.so.1.0` | ROS 与底层日志桥 | [ros/console_bridge](https://github.com/ros/console_bridge) |
| `libPocoFoundation.so.88` | `class_loader` 的 `dlopen` 后端 | [pocoproject/poco](https://github.com/pocoproject/poco) |
| `libtinyxml2.so.8` | XML 解析 | [leethomason/tinyxml2](https://github.com/leethomason/tinyxml2) |
| `libuuid.so.1` | UUID（bondcpp） | 本仓库 `src/libuuid` |

`libc` / `libstdc++` 等系统库不打包。

## `python/`

给 `roscore`、`rosmaster`、`rosnode`、`rosrun` 用的隔离 Python 环境（含 ROS 生成的 msg/srv）。`run.sh` 会把 `PYTHONPATH` 指到这里。具体模块不再展开。

## `share/`

官方 `pluginlib` / `rosrun` / `roslaunch` 按包名找描述文件。至少需要：

| 目录 | 作用 | 来源 |
|------|------|------|
| `my_nodelet_plugin/` | `package.xml` + `nodelet_plugins.xml`（Talker / Listener 类名） | 本仓库 `src/my_nodelet_plugin` |
| `nodelet/` | 官方 nodelet 包清单 | [ros/nodelet_core](https://github.com/ros/nodelet_core) |
| `rosout/` / `roslaunch/` / `ros/` | `roscore` 找 `rosout`、读 `roscore.xml` | [ros/ros_comm](https://github.com/ros/ros_comm)、[ros/ros](https://github.com/ros/ros) |
| `pluginlib/` / `class_loader/` / `rospack/` | 插件与包查找 | 对应 ROS 仓库 |
| `roscpp/` / `std_msgs/` / `rosgraph_msgs/` / `bond/` | 消息/依赖清单 | 对应 ROS 仓库 |
