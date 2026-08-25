# isolated_ros_nodelet

在 Ubuntu PC 上从源码编出一套**完全隔离**的 ROS Noetic + Nodelet 环境：不写 `/opt/ros`、不装系统 `ros-*` / `libboost-*-dev`，产物都在仓库目录里。

三条路径：

| | 官方全量 | 定制最小 ROS 环境 | 应用层 |
|--|----------|-------------------|--------|
| 脚本 | `official_full.sh` | `custom_mini.sh` | `app/*/make.sh` |
| 作用 | 官方 `roscore` + `rosrun nodelet` demo | 只编 / 打开隔离 ROS 环境 | Talker/Listener（依赖 custom_mini） |
| 运行目录 | `official_full_runtime/` | `custom_mini_runtime/`（交互 shell） | `app_runtime/` |

目标：在 PC 上验证 Talker / Listener 进程内通信；源码可拷到 ARM Linux（如 RV1126）后重新编译。本仓库不提供交叉工具链。

`src/` 里大部分目录由 build 脚本从网上拉取，**为方便也纳入 git**。某包有问题可删掉对应目录再跑 `build`，脚本会补下缺失源码。应用层源码在 `app/`。

## 特性

- 隔离前缀：官方全量进 `official_full_install/`；定制最小 ROS 进 `custom_mini_install/`；应用层产物进 `app_runtime/`
- `custom_mini.sh` 只负责 ROS 栈；应用逻辑由 `app/*/make.sh` 编译并安装到 `app_runtime/`
- `package` 抽出可拷走的 runtime，不依赖整棵 install
- Python 3.12 兼容层（`python_compat/`），无需系统 ROS

## 系统要求

脚本允许的系统工具：`g++`、`cmake`、`make`、`python3`、`curl`、`tar`（可选 `git`）。当前按 **Ubuntu 24.04 / Python 3.12 / x86_64** 验证。`official_full.sh` / `custom_mini.sh` 里 pip 目标目录写死为 `lib/python3.12/site-packages`；换发行版需改这一处。

- `build-essential`（`g++`、`make`）、`cmake` ≥ 3.0、`python3`、`curl`、`tar`
- 磁盘建议 8GB+，内存建议 4GB+（8GB 更从容）
- 本机无 `python3-dev` 时，脚本会 `apt-get download libpythonX.Y-dev` 再解压头文件（不 `apt install`）

**不要**安装 `ros-*`、`python3-ros*`、`libboost-*-dev`。脚本不会、也不得执行这些安装。

## 快速开始

```bash
chmod +x official_full.sh custom_mini.sh app/*/make.sh

# 官方全量（首次约 15–30 分钟；脚本不编 app/）
./official_full.sh build
ROS_INSTALL=$PWD/official_full_install (cd app/talker_nodelet && ./make.sh x86)
ROS_INSTALL=$PWD/official_full_install (cd app/listener_nodelet && ./make.sh x86)
./official_full.sh package
./official_full.sh run

# 定制最小：先编 ROS 环境，再编 / 安装应用层
./custom_mini.sh build
./custom_mini.sh package
./custom_mini.sh run          # 打开 ROS 环境交互 shell（不是 demo）

(cd app/talker_nodelet && ./make.sh x86 && ./make.sh install)
(cd app/listener_nodelet && ./make.sh x86 && ./make.sh install)
(cd app/custom_ros_nodelet && ./make.sh x86 && ./make.sh install)
./app_runtime/run.sh          # Talker / Listener demo
```

`./app_runtime/run.sh` 成功时终端周期性出现：

```text
[INFO] [...] Publishing: Hello from Nodelet 0
[INFO] [...] Received: Hello from Nodelet 0
```

Ctrl+C 结束。日志分别在 `official_full_logs/`、`custom_mini_logs/`。

runtime 说明见：

- [doc/official_full/README.md](doc/official_full/README.md)
- [doc/custom_mini/README.md](doc/custom_mini/README.md)
- [doc/app/README.md](doc/app/README.md)

## Stage 仿真（stage_ros，独立于 src/）

`tool/` 是仓库的**工具集**，目前只含一个工具 **stage_ros**（Stage 机器人仿真环境），
与 `src/` 组件无关：每个工具自包含在 `tool/<工具>/` 下（源码、独立编译/运行脚本、
README 都在里面），只依赖 `custom_mini.sh build` 编出的隔离 ROS 环境。
Stage 需要 FLTK 等系统库，tool 工具**允许 apt 安装**（清单见
[tool/stage_ros/README.md](tool/stage_ros/README.md)）。

```bash
./custom_mini.sh build             # 隔离 ROS 环境（首次，约 15–30 分钟）
./tool/stage_ros/build.sh deps     # apt 装 FLTK/JPEG/PNG/OpenGL 等系统依赖（首次）
./tool/stage_ros/build.sh          # 编译 Stage + stage_ros
./run_stage.sh                     # 默认弹出 Stage GUI 仿真窗口
./run_stage.sh --headless          # 无界面（CI / 无显示器）
```

环境用 Stage 版 `box_house.world`，机器人模型用 `xacro/mycar.urdf.xacro`（渲染成 URDF 发
`/robot_description`）。工具总览见 [tool/README.md](tool/README.md)，stage_ros 详见
[tool/stage_ros/README.md](tool/stage_ros/README.md)。

## 仓库结构

```text
isolated_ros_nodelet/
├── official_full.sh         # 官方全量：build / package / run / clean / help
├── custom_mini.sh           # 定制最小 ROS 环境（不含应用层）
├── app/                     # 应用层源码
│   ├── custom_ros_nodelet/  # JSON 驱动的 manager（include/nlohmann/json.hpp；./make.sh）
│   ├── talker_nodelet/      # 发送端软件包（./make.sh x86|install）
│   └── listener_nodelet/    # 接收端软件包（./make.sh x86|install）
├── doc/
│   ├── official_full/
│   ├── custom_mini/         # env.sh / run.sh（打开 ROS shell）
│   └── app/                 # run.sh / plugins.json
├── python_compat/
├── src/                     # ROS / 第三方源码（libuuid 等）
├── tool/                    # 工具集（每个工具自包含子目录；总览见 tool/README.md）
│   └── stage_ros/           # Stage 仿真（stage_ros，独立编译/运行脚本；见 tool/stage_ros/README.md）
├── official_full_{install,build,runtime,logs}/
├── custom_mini_{install,build,runtime,logs}/
└── app_runtime/             # 应用产物（各 make.sh install）
```

`custom_mini.sh run` 打开已注入环境的交互式 bash；应用 demo 用 `./app_runtime/run.sh`。`official_full.sh` / `custom_mini.sh` 都**不编** `app/`；插件由各包 `make.sh` 自行编译。

## official_full.sh build 做什么

无人值守（`set -e`，出错即停并留日志）：

1. **隔离前缀**：产物写入 `official_full_install/`，环境变量只指向该目录。
2. **Python 依赖**：`get-pip.py` 把 pip 装进 install，再装 `catkin_pkg`、`empy==3.3.4`、`PyYAML`、`rospkg` 等。不写系统 `site-packages`。
3. **第三方库**：Boost、tinyxml2、console_bridge、Poco、本仓库 `libuuid`，全部编进 install。
4. **ROS 源码**：从 GitHub 拉（优先 `noetic-devel`）。
5. **`catkin_make_isolated --install`**：只编 ROS 核心。关测试，`ROSCONSOLE_BACKEND=print`。
6. **日志**：`official_full_logs/build.log` 与分步 `01_python.log` … `09_tree.log`。

应用层另做：
```bash
ROS_INSTALL=$PWD/official_full_install (cd app/talker_nodelet && ./make.sh x86)
ROS_INSTALL=$PWD/official_full_install (cd app/listener_nodelet && ./make.sh x86)
./official_full.sh package
```

可重复执行：已装好的 Boost / 第三方会跳过。

`package` 从 install **抽出** `bin/` `lib/` `python/` `share/`，不是整棵拷贝，也不再依赖 install 才能跑。

`run`（`official_full_runtime/run.sh`）只设 runtime 自己的环境变量，然后：

1. 启动 `roscore`，等到 `/rosout`
2. `rosrun nodelet nodelet manager`
3. load `talker_nodelet/TalkerNodelet` 与 `listener_nodelet/ListenerNodelet`

Talker 每秒在 `chatter` 发 `std_msgs/String`；Listener 订阅并打印。两边日志交替即表示进程内通信正常。

隔离自检：

```bash
ls /opt/ros || echo "no /opt/ros"
ldd official_full_install/lib/libtalker_nodelet.so
ldd official_full_install/lib/liblistener_nodelet.so
```

`libboost_*`、`libconsole_bridge`、`libPocoFoundation`、`libtinyxml2`、ROS 库应来自本仓库 `official_full_install/lib`。`libc` / `libstdc++` 等是系统 C/C++ 运行时，属预期。

## 本仓库编写的代码

克隆即可得到，**不要从网上下载覆盖**。

| 路径 | 作用 |
|------|------|
| `official_full.sh` | 官方全量：`build` → install，`package` → runtime |
| `custom_mini.sh` | 定制最小 ROS 环境：`build` → `custom_mini_install/`，`run` → 交互 shell |
| `app/custom_ros_nodelet/make.sh` | manager：`x86` 编译，`install` → `app_runtime/bin/` |
| `app/talker_nodelet/make.sh` | 发送端：`x86` 编译，`install` → `app_runtime/lib/` |
| `app/listener_nodelet/make.sh` | 接收端：`x86` 编译，`install` → `app_runtime/lib/` |
| `doc/official_full/` | 官方 runtime 的 `run.sh` / README |
| `doc/custom_mini/` | ROS 环境 runtime 的 `env.sh` / `run.sh` / README |
| `doc/app/` | 应用 runtime 的 `run.sh` / `plugins.json` / README |
| `python_compat/sitecustomize.py` | 补回 `collections.Mapping`、`inspect.getargspec`，并 `import setuptools` 恢复 `distutils` |
| `python_compat/imp.py` | 给仍 `import imp` 的 ROS 脚本提供 `load_source` |
| `src/libuuid/` | 兼容 `uuid/uuid.h` 的小型共享库，避免系统 `uuid-dev` |
| `app/talker_nodelet/` | 发送端 Nodelet（独立 CMake，编译不依赖 `package.xml`） |
| `app/listener_nodelet/` | 接收端 Nodelet（同上） |
| `app/custom_ros_nodelet/` | JSON 驱动的 manager（无 catkin） |

### `app/talker_nodelet` / `app/listener_nodelet`

| 文件 | 作用 |
|------|------|
| `CMakeLists.txt` | 独立 CMake（`find_package(roscpp/nodelet/...)`，无 catkin） |
| `nodelet_plugins.xml` | pluginlib 注册本包类名（官方路径用） |
| `package.xml` | 可选；仅官方 `nodelet load` 的 share 发现需要，**编译不依赖** |
| `src/*.cpp` / `include/...` | Talker 或 Listener 实现 |
| `make.sh` | `x86` / `install` / `clean` → `app_runtime/lib/` |

两个 Nodelet 由同一个 manager `dlopen`，同一进程，话题走进程内传输。

### `src/libuuid`

| 文件 | 作用 |
|------|------|
| `include/uuid/uuid.h` | 与 util-linux libuuid 相近的 C API |
| `uuid.c` | `/dev/urandom` 生成 UUID v4 |
| `CMakeLists.txt` | 编 `libuuid.so`，由各脚本装进各自的 `*_install/` |

### `app/custom_ros_nodelet`

| 文件 | 作用 |
|------|------|
| `CMakeLists.txt` | 独立 CMake 工程，链接 `custom_mini_install` 里的 roscpp / nodelet / class_loader |
| `src/custom_ros_nodelet_manager.cpp` | 读 `plugins.json`，按 `.so` 路径实例化 Nodelet |
| `include/nlohmann/json.hpp` | 单头 JSON（本包自带） |
| `make.sh` | `x86` / `install` / `clean` |

先 `./custom_mini.sh build`，再对各应用包 `./make.sh x86 && ./make.sh install`。
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
分别 `cd app/talker_nodelet` / `app/listener_nodelet`（以及需要时的 `custom_ros_nodelet`）后 `./make.sh x86`。对着官方 install 时加 `ROS_INSTALL=$PWD/official_full_install`；定制路径默认用 `custom_mini_install`，再 `./make.sh install` → `app_runtime/`。`official_full.sh` / `custom_mini.sh` 都不会编 `app/`。
