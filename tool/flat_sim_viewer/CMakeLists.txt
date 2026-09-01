# =============================================================================
# flat_sim_viewer —— SLAM 建图查看工具（C++ / Qt6 Widgets / CMake）
# 对外入口是 tool/flat_sim_viewer/build.sh（会注入隔离 ROS 环境）；一般不手敲 cmake。
#
# 布局：view（Qt6 自绘，不依赖 ROS）/ app（主窗口控制器）/ ros（SlamListener 拷贝）
# 查看器必有 UI：Qt6 为 REQUIRED（无 headless 形态）。
# =============================================================================
cmake_minimum_required(VERSION 3.16)
project(flat_sim_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)  # Q_OBJECT 类（SlamView / ViewerWindow）的 moc 处理

find_package(Qt6 6.4 REQUIRED COMPONENTS Widgets)

# ---- 隔离 ROS（custom_mini_install；由 build.sh 通过 CMAKE_PREFIX_PATH 注入）----
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag REQUIRED)          # 回放：读 map_log_nodelet 录的 .bag
find_package(rosbag_storage REQUIRED)

# Boost 用系统版本（CMAKE_PREFIX_PATH 前置了隔离 ROS，那里也有 Boost 1.71 的
# cmake config；带最低版本号 1.83 才能过滤掉它、命中系统 /usr 的 Boost）
find_package(Boost 1.83 REQUIRED COMPONENTS system thread chrono)
find_package(Threads REQUIRED)

set(VIEWER_MSG_PKGS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs rosbag rosbag_storage)
set(VIEWER_ROS_INCLUDES)
set(VIEWER_ROS_LIBS)
foreach(_pkg ${VIEWER_MSG_PKGS})
  list(APPEND VIEWER_ROS_INCLUDES ${${_pkg}_INCLUDE_DIRS})
  list(APPEND VIEWER_ROS_LIBS ${${_pkg}_LIBRARIES})
endforeach()

# 注意：catkin 的 Config.cmake 会把全局 BUILD_SHARED_LIBS 置 ON，
# 这里显式 STATIC（与 tool/flat_sim 的做法一致）。

# ---- ros：SlamListener（实时源）+ SnapshotBuilder（消息→快照，实时/回放共用）
#      + replay/BagPlayer（rosbag 回放引擎）----
add_library(flat_viewer_ros STATIC
  src/ros/SlamListener.cpp
  src/ros/SnapshotBuilder.cpp
  src/replay/BagPlayer.cpp)
target_include_directories(flat_viewer_ros PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${VIEWER_ROS_INCLUDES} ${Boost_INCLUDE_DIRS})
target_link_libraries(flat_viewer_ros PUBLIC ${VIEWER_ROS_LIBS} ${Boost_LIBRARIES}
                                             Threads::Threads)

# ---- flat_sim_viewer ----
add_executable(flat_sim_viewer src/main.cpp src/app/ViewerWindow.cpp src/view/SlamView.cpp)
target_compile_definitions(flat_sim_viewer PRIVATE QT_NO_KEYWORDS=1)
target_link_libraries(flat_sim_viewer PRIVATE flat_viewer_ros Qt6::Widgets)

install(TARGETS flat_sim_viewer RUNTIME DESTINATION bin)
