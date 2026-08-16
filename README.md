# isolated_ros_nodelet

在 Ubuntu PC 上从源码编出一套**完全隔离**的 ROS Noetic + Nodelet 环境：不写 `/opt/ros`、不装系统 `ros-*` / `libboost-*-dev`，产物都在仓库目录里。

两条可运行路径：

| | 官方全量 | 定制最小 |
|--|----------|----------|
| 脚本 | `official_full.sh` | `custom_mini.sh` |
| 启动方式 | `roscore` + `rosrun nodelet` | `rosmaster` + `custom_mini_manager` + `plugins.json` |
| 是否需要 `package.xml` / `nodelet_plugins.xml` | 需要 | 不需要 |
| 运行目录 | `official_full_runtime/` | `custom_mini_runtime/` |

目标：在 PC 上验证 Talker / Listener 进程内通信；源码可拷到 ARM Linux（如 RV1126）后重新编译。本仓库不提供交叉工具链。

`src/` 里大部分目录由 `official_full.sh build` 从网上拉取，**为方便也纳入 git**。某包有问题可删掉对应目录再跑 `build`，脚本会补下缺失源码。

## 特性

- 隔离前缀：官方全量的库、头文件、Python 包装进 `official_full_install/`；`custom_mini_manager` 装进 `custom_mini_install/`
- 官方 demo 与最小 demo 各有独立的 `build` / `package` / `run` / `clean`
- `package` 抽出可拷走的 runtime（`bin/` `lib/` `python/`，官方路径另有 `share/`），不依赖整棵 install
- Python 3.12 兼容层（`python_compat/`），无需系统 ROS

## 系统要求

脚本允许的系统工具：`g++`、`cmake`、`make`、`python3`、`curl`、`tar`（可选 `git`）。当前按 **Ubuntu 24.04 / Python 3.12 / x86_64** 验证。`official_full.sh` 里 pip 目标目录写死为 `lib/python3.12/site-packages`；换发行版需改这一处。

- `build-essential`（`g++`、`make`）、`cmake` ≥ 3.0、`python3`、`curl`、`tar`
- 磁盘建议 8GB+，内存建议 4GB+（8GB 更从容）
- 本机无 `python3-dev` 时，脚本会 `apt-get download libpythonX.Y-dev` 再解压头文件（不 `apt install`）

**不要**安装 `ros-*`、`python3-ros*`、`libboost-*-dev`。脚本不会、也不得执行这些安装。

## 快速开始

```bash
chmod +x official_full.sh custom_mini.sh

# 1) 官方全量（首次约 15–30 分钟）
./official_full.sh build
./official_full.sh package
./official_full.sh run

# 2) 定制最小（依赖上一步编好的 official_full_install/）
./custom_mini.sh build
./custom_mini.sh package
./custom_mini.sh run
```

成功时终端周期性出现：

```text
[INFO] [...] Publishing: Hello from Nodelet 0
[INFO] [...] Received: Hello from Nodelet 0
```

Ctrl+C 结束。日志分别在 `official_full_logs/`、`custom_mini_logs/`。

两个 runtime 里各文件的来源见：

- [doc/official_full/README.md](doc/official_full/README.md)
- [doc/custom_mini/README.md](doc/custom_mini/README.md)

## 仓库结构

```text
isolated_ros_nodelet/
├── official_full.sh         # 官方全量：build / package / run / clean / help
├── custom_mini.sh           # 定制最小：同样五个子命令
├── doc/
│   ├── official_full/       # 官方 runtime 的 README.md / run.sh（runtime 里是软链接）
│   └── custom_mini/         # 最小 runtime 的 README.md / run.sh / plugins.json
├── python_compat/           # 本仓库：Python 3.12 兼容层
├── src/
│   ├── my_nodelet_plugin/   # 本仓库：Talker / Listener
│   ├── custom_mini/         # 本仓库：JSON 驱动的 manager
│   ├── libuuid/             # 本仓库：最小 libuuid
│   ├── nlohmann_json/       # custom_mini 用的单头文件（build 时下载）
│   └── …                   # 下载的 Boost / ROS / 第三方（见下表）
├── official_full_install/   # 官方全量安装产物（不入库）
├── official_full_build/     # 官方全量中间文件（不入库）
├── official_full_runtime/   # 官方全量可拷贝运行目录（不入库）
├── official_full_logs/      # 官方全量编译日志（不入库）
├── custom_mini_install/     # 定制最小安装产物（不入库）
├── custom_mini_build/       # 定制最小中间文件（不入库）
├── custom_mini_runtime/     # 定制最小可拷贝运行目录（不入库）
└── custom_mini_logs/        # 定制最小编译日志（不入库）
```

五个子命令两边相同：`build` 编译，`package` 抽 runtime，`run` 跑 demo，`clean` 只删 `*_build/`（保留 install、runtime、logs），无参数 = `help`。

## official_full.sh build 做什么

无人值守（`set -e`，出错即停并留日志）：

1. **隔离前缀**：产物写入 `official_full_install/`，环境变量只指向该目录。
2. **Python 依赖**：`get-pip.py` 把 pip 装进 install，再装 `catkin_pkg`、`empy==3.3.4`、`PyYAML`、`rospkg` 等。不写系统 `site-packages`。
3. **第三方库**：Boost、tinyxml2、console_bridge、Poco、本仓库 `libuuid`，全部编进 install。
4. **ROS 源码**：从 GitHub 拉（优先 `noetic-devel`）。
5. **`catkin_make_isolated --install`**：编 ROS 核心与 `my_nodelet_plugin`。关测试，`ROSCONSOLE_BACKEND=print`。
6. **日志**：`official_full_logs/build.log` 与分步 `01_python.log` … `09_tree.log`。

可重复执行：已装好的 Boost / 第三方会跳过。

`package` 从 install **抽出** `bin/` `lib/` `python/` `share/`，不是整棵拷贝，也不再依赖 install 才能跑。

`run`（`official_full_runtime/run.sh`）只设 runtime 自己的环境变量，然后：

1. 启动 `roscore`，等到 `/rosout`
2. `rosrun nodelet nodelet manager`
3. load `my_nodelet_plugin/TalkerNodelet` 与 `ListenerNodelet`

Talker 每秒在 `chatter` 发 `std_msgs/String`；Listener 订阅并打印。两边日志交替即表示进程内通信正常。

隔离自检：

```bash
ls /opt/ros || echo "no /opt/ros"
ldd official_full_install/lib/libmy_nodelet_plugin.so
```

`libboost_*`、`libconsole_bridge`、`libPocoFoundation`、`libtinyxml2`、ROS 库应来自本仓库 `official_full_install/lib`。`libc` / `libstdc++` 等是系统 C/C++ 运行时，属预期。

## 本仓库编写的代码

克隆即可得到，**不要从网上下载覆盖**。

| 路径 | 作用 |
|------|------|
| `official_full.sh` | 官方全量：`build` → install，`package` → runtime |
| `custom_mini.sh` | 定制最小：`build` → `custom_mini_install/`，`package` → runtime |
| `doc/official_full/` | 官方 runtime 的 `run.sh` / README |
| `doc/custom_mini/` | 最小 runtime 的 `run.sh` / `plugins.json` / README |
| `python_compat/sitecustomize.py` | 补回 `collections.Mapping`、`inspect.getargspec`，并 `import setuptools` 恢复 `distutils` |
| `python_compat/imp.py` | 给仍 `import imp` 的 ROS 脚本提供 `load_source` |
| `src/libuuid/` | 兼容 `uuid/uuid.h` 的小型共享库，避免系统 `uuid-dev` |
| `src/my_nodelet_plugin/` | 演示用 Nodelet 插件 |
| `src/custom_mini/` | JSON 驱动的 manager（无 `package.xml`，catkin 扫不到） |

### `src/my_nodelet_plugin`

| 文件 | 作用 |
|------|------|
| `package.xml` | catkin 清单；`<export><nodelet plugin=.../></export>` 供官方 manager 找插件 |
| `CMakeLists.txt` | 编 `libmy_nodelet_plugin.so`，安装头文件与 `nodelet_plugins.xml` |
| `nodelet_plugins.xml` | pluginlib 注册：`TalkerNodelet` / `ListenerNodelet` |
| `include/.../talker_nodelet.h` | Talker：定时器 + Publisher |
| `include/.../listener_nodelet.h` | Listener：订阅 `chatter` |
| `src/talker_nodelet.cpp` | 每秒发布 `std_msgs/String`；`PLUGINLIB_EXPORT_CLASS` |
| `src/listener_nodelet.cpp` | 打印收到的消息 |

两个 Nodelet 由同一个 manager `dlopen`，同一进程，话题走进程内传输。

### `src/libuuid`

| 文件 | 作用 |
|------|------|
| `include/uuid/uuid.h` | 与 util-linux libuuid 相近的 C API |
| `uuid.c` | `/dev/urandom` 生成 UUID v4 |
| `CMakeLists.txt` | 编 `libuuid.so` 装到 `official_full_install/` |

### `src/custom_mini`

| 文件 | 作用 |
|------|------|
| `CMakeLists.txt` | 独立 CMake 工程，链接 `official_full_install` 里的 roscpp / nodelet / class_loader |
| `src/custom_mini_manager.cpp` | 读 `plugins.json`，按 `.so` 路径实例化 Nodelet |

头文件 `nlohmann/json.hpp` 由 `custom_mini.sh build` 下载到 `src/nlohmann_json/`。

## 网上下载的软件包

缺失时 `official_full.sh build` 自动拉取。无网络可按「建议 clone」预先放到 `src/<目录>`；目录非空则跳过下载。脚本会按 `official_full.sh` 里 `fetch_github` 的分支参数**从左到右尝试**（先 `noetic-devel`，没有再试表中其它分支）。下表 clone 命令写的是通常能成功的那个。

### 第三方库

| `src/` 目录 | 用途 | git / 下载地址 | 建议 clone |
|-------------|------|----------------|------------|
| `boost/` | roscpp、rospack 等需要的 Boost 1.71.0 | [archives.boost.io](https://archives.boost.io/release/1.71.0/source/boost_1_71_0.tar.gz)（失败则试 SourceForge） | 脚本下 tar 到 `src/boost/boost_1_71_0.tar.gz`。若坚持 git：`git clone --branch boost-1.71.0 --recursive https://github.com/boostorg/boost.git src/boost/boost_1_71_0` |
| `tinyxml2/` | pluginlib、rospack 解析 XML | [leethomason/tinyxml2](https://github.com/leethomason/tinyxml2.git) | `git clone --branch 8.0.0 https://github.com/leethomason/tinyxml2.git src/tinyxml2` |
| `console_bridge/` | ROS 与底层日志桥 | [ros/console_bridge](https://github.com/ros/console_bridge.git) | `git clone --branch 1.0.2 https://github.com/ros/console_bridge.git src/console_bridge` |
| `poco/` | class_loader 用 Poco Foundation 做 `dlopen` | [pocoproject/poco](https://github.com/pocoproject/poco.git) | `git clone --branch poco-1.11.8-release https://github.com/pocoproject/poco.git src/poco` |
| `python_dev/` | 解压 `libpythonX.Y-dev` 的 `Python.h`（本机无对应头文件时） | 非 git：`apt-get download libpython$(python3 -c 'import sys; print("%d.%d"%sys.version_info[:2])')-dev` | 无需 clone |
| `nlohmann_json/` | `custom_mini_manager` 解析 JSON | [nlohmann/json v3.11.3 单头](https://github.com/nlohmann/json) | `custom_mini.sh build` 自动下载 |

### ROS / 消息生成

| `src/` 目录 | 用途 | git 地址 | 建议 clone |
|-------------|------|----------|------------|
| `catkin/` | `catkin_make_isolated` | [ros/catkin](https://github.com/ros/catkin.git) | `git clone --branch noetic-devel https://github.com/ros/catkin.git src/catkin` |
| `cmake_modules/` | FindUUID、FindTinyXML2 等 | [ros/cmake_modules](https://github.com/ros/cmake_modules.git) | `git clone --branch 0.5-devel https://github.com/ros/cmake_modules.git src/cmake_modules` |
| `genmsg/` | 消息生成框架 | [ros/genmsg](https://github.com/ros/genmsg.git) | `git clone --branch noetic-devel https://github.com/ros/genmsg.git src/genmsg` |
| `gencpp/` | 生成 C++ 消息头 | [ros/gencpp](https://github.com/ros/gencpp.git) | `git clone --branch noetic-devel https://github.com/ros/gencpp.git src/gencpp` |
| `genpy/` | 生成 Python 消息模块 | [ros/genpy](https://github.com/ros/genpy.git) | `git clone --branch noetic-devel https://github.com/ros/genpy.git src/genpy` |
| `geneus/` | EusLisp 消息（message_generation 依赖） | [jsk-ros-pkg/geneus](https://github.com/jsk-ros-pkg/geneus.git) | `git clone --branch master https://github.com/jsk-ros-pkg/geneus.git src/geneus` |
| `gennodejs/` | Node.js 消息 | [RethinkRobotics-opensource/gennodejs](https://github.com/RethinkRobotics-opensource/gennodejs.git) | `git clone --branch kinetic-devel https://github.com/RethinkRobotics-opensource/gennodejs.git src/gennodejs` |
| `genlisp/` | Lisp 消息 | [ros/genlisp](https://github.com/ros/genlisp.git) | `git clone --branch kinetic-devel https://github.com/ros/genlisp.git src/genlisp` |
| `message_generation/` | 把各语言生成器绑在一起 | [ros/message_generation](https://github.com/ros/message_generation.git) | `git clone --branch kinetic-devel https://github.com/ros/message_generation.git src/message_generation` |
| `message_runtime/` | 消息运行时依赖声明 | [ros/message_runtime](https://github.com/ros/message_runtime.git) | `git clone --branch kinetic-devel https://github.com/ros/message_runtime.git src/message_runtime` |
| `std_msgs/` | `std_msgs/String` 等 | [ros/std_msgs](https://github.com/ros/std_msgs.git) | `git clone --branch kinetic-devel https://github.com/ros/std_msgs.git src/std_msgs` |
| `ros_comm_msgs/` | `rosgraph_msgs`、`std_srvs` | [ros/ros_comm_msgs](https://github.com/ros/ros_comm_msgs.git) | `git clone --branch noetic-devel https://github.com/ros/ros_comm_msgs.git src/ros_comm_msgs` |
| `roscpp_core/` | `cpp_common`、`rostime`、序列化 | [ros/roscpp_core](https://github.com/ros/roscpp_core.git) | `git clone --branch noetic-devel https://github.com/ros/roscpp_core.git src/roscpp_core` |
| `rosconsole/` | `ROS_INFO` 等日志 | [ros/rosconsole](https://github.com/ros/rosconsole.git) | `git clone --branch noetic-devel https://github.com/ros/rosconsole.git src/rosconsole` |
| `ros/` | `roslib`、`rosclean`、`rosbash` 等 | [ros/ros](https://github.com/ros/ros.git) | `git clone --branch noetic-devel https://github.com/ros/ros.git src/ros` |
| `rospack/` | 按包名查找 share/lib | [ros/rospack](https://github.com/ros/rospack.git) | `git clone --branch noetic-devel https://github.com/ros/rospack.git src/rospack` |
| `ros_comm/` | `roscpp`、`rospy`、`roslaunch`、`roscore`、`xmlrpcpp` | [ros/ros_comm](https://github.com/ros/ros_comm.git) | `git clone --branch noetic-devel https://github.com/ros/ros_comm.git src/ros_comm` |
| `pluginlib/` | 按 XML 描述动态加载 C++ 类 | [ros/pluginlib](https://github.com/ros/pluginlib.git) | `git clone --branch noetic-devel https://github.com/ros/pluginlib.git src/pluginlib` |
| `class_loader/` | 插件 `.so` 的加载器 | [ros/class_loader](https://github.com/ros/class_loader.git) | `git clone --branch noetic-devel https://github.com/ros/class_loader.git src/class_loader` |
| `nodelet_core/` | `nodelet` 管理器与基类 | [ros/nodelet_core](https://github.com/ros/nodelet_core.git) | `git clone --branch noetic-devel https://github.com/ros/nodelet_core.git src/nodelet_core` |
| `bond_core/` | Nodelet 与 loader 之间的心跳 | [ros/bond_core](https://github.com/ros/bond_core.git) | `git clone --branch noetic-devel https://github.com/ros/bond_core.git src/bond_core` |

说明：

- `geneus` 在 `jsk-ros-pkg`，`gennodejs` 在 `RethinkRobotics-opensource`。
- `std_msgs`、`message_generation` 等仓库往往没有单独的 `noetic-devel`，脚本会落到 `kinetic-devel`。
- 为缩短编译、避开 lz4/bzip2，脚本会给这些包打 `CATKIN_IGNORE`：`rosbag`、`rosbag_storage`、`roslz4`、`topic_tools`、`roswtf`、`nodelet_topic_tools`、`test_nodelet`、`test_nodelet_topic_tools`、`nodelet_tutorial_math`、`pluginlib_tutorials`、`test_bond`、`roscreate`、`rosmake`、`mk`、`rosbuild`、`rosboost_cfg`、`roscpp_tutorials`、`rospy_tutorials`，以及 `ros_comm/test`、`cmake_modules/tests`。

## 移植到 ARM Linux

保证**源码结构可移植**，不提供交叉脚本或预编译 ARM 二进制。目标机需要：

- `g++`（建议 C++14）、`cmake` ≥ 3.0、`make`、`python3`、`curl` / `git`
- 系统库：`libstdc++`、`libpthread`、`libdl`、`libm`、套接字（一般在 libc 里）
- Boost、console_bridge、tinyxml2、Poco、uuid **不要**用发行版开发包
- 存储建议 2GB+；内存不足时把 `official_full.sh` 里的 `JOBS="$(nproc 2>/dev/null || echo 4)"` 改成 `JOBS=1`
- 首次可能下 Boost（约 100MB）和 GitHub 源码；离线则把 tar / clone 预先放进 `src/`
- 跑 `roscore` 需要 Python3；无 Python 时把 Master 放别的机器、用 `ROS_MASTER_URI` 连——这属于部署，不在本项目范围

RV1126 一类设备内核需 Linux 4.19+。在目标架构上重新 `./official_full.sh build`，不要混用 x86_64 的 `official_full_install/`。

## 常见问题

**`./official_full.sh build` 在某一步失败？**  
看 `official_full_logs/build.log` 和对应 `01_python.log` … `09_tree.log` 末尾。修好后可再跑（已完成的 Boost/第三方会跳过）。`custom_mini` 同理看 `custom_mini_logs/`。

**提示找不到 `python`？**  
脚本会在 `official_full_install/bin/python` 链到 `python3`。请用 `./official_full.sh run` 启动。

**找不到 `em` / `catkin_pkg`？**  
确认编译时的 `PYTHONPATH` 含 `official_full_install/lib/python3.12/site-packages`（脚本写死的 pip 目录）与 `lib/python3/dist-packages`。

**Listener 收不到消息？**  
确认 Talker / Listener 在同一个 manager 里。官方路径会等 `/rosout` 与 manager。runtime 里有 `rosnode list`；`rostopic` 只在 `official_full_install/bin/`，没有打进 runtime。

**`ldd` 仍显示系统 `libboost`？**  
不要装 `libboost-*-dev`，删 `official_full_build` 后重编，并用对应的 `*.sh run` 启动。

**Ubuntu 24.04 / Python 3.12 能否用？**  
可以。靠 `python_compat` 和 C++14 / Boost 1.71 源码编译。

**能否 `sudo apt install ros-noetic-*` 来加速？**  
不能，会破坏隔离。

**如何只重编插件？**  
环境已指向 `official_full_install/` 时，可在 `official_full_build/isolated/my_nodelet_plugin` 执行 `make install`，或删掉该包 build 目录后重跑 `./official_full.sh build`。
