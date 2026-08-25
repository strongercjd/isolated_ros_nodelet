# =============================================================================
# flat_sim —— 自研 2D 机器人仿真（C++ / CMake，无 TF / 无 URDF）
# 对外入口是 tool/flat_sim/build.sh（会注入隔离 ROS 环境）；一般不手敲 cmake。
#
# 布局：core（纯算法）/ format（自研格式解析）/ gui（SDL2，可选）/ ros（话题桥接）
# =============================================================================
cmake_minimum_required(VERSION 3.10)
project(flat_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(FLAT_SIM_ENABLE_GUI "构建 2D 俯视 GUI（需要 SDL2；找不到则自动仅编 headless）" ON)

# ---- 隔离 ROS（custom_mini_install；由 build.sh 通过 CMAKE_PREFIX_PATH 注入）----
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Boost 只在隔离 ROS 内找（系统未装 libboost-dev；build.sh 已设 BOOST_ROOT）
if(NOT DEFINED Boost_NO_SYSTEM_PATHS)
  set(Boost_NO_SYSTEM_PATHS ON)
endif()
find_package(Boost REQUIRED COMPONENTS system thread chrono)
find_package(Threads REQUIRED)

set(FLAT_SIM_MSG_PKGS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs)
set(FLAT_SIM_ROS_INCLUDES)
set(FLAT_SIM_ROS_LIBS)
foreach(_pkg ${FLAT_SIM_MSG_PKGS})
  list(APPEND FLAT_SIM_ROS_INCLUDES ${${_pkg}_INCLUDE_DIRS})
  list(APPEND FLAT_SIM_ROS_LIBS ${${_pkg}_LIBRARIES})
endforeach()

# 注意：catkin 的 Config.cmake 会把全局 BUILD_SHARED_LIBS 置 ON，
# 这里各库显式 STATIC：flat_sim_node 链成单文件可执行，安装后无额外 so 依赖。

# ---- core：几何 / 世界 / 仿真（不依赖 ROS 与 GUI）----
add_library(flat_sim_core STATIC src/core/Geometry.cpp src/core/Simulator.cpp)
target_include_directories(flat_sim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ---- format：自研 .fworld / .frobot 解析（# 备注在解析层丢弃）----
add_library(flat_sim_format STATIC src/format/TextFormat.cpp src/format/WorldLoader.cpp)
target_link_libraries(flat_sim_format PUBLIC flat_sim_core)

# ---- gui：SDL2 2D 俯视图（可选；headless 构建完全不链 SDL）----
set(FLAT_SIM_HAVE_GUI OFF)
if(FLAT_SIM_ENABLE_GUI)
  find_package(SDL2 QUIET)
  if(SDL2_FOUND OR TARGET SDL2::SDL2)
    set(FLAT_SIM_HAVE_GUI ON)
    add_library(flat_sim_gui STATIC src/gui/Gui.cpp src/gui/SlamView.cpp)
    target_link_libraries(flat_sim_gui PUBLIC flat_sim_core)
    target_compile_definitions(flat_sim_gui PUBLIC FLAT_SIM_HAVE_GUI=1)
    if(TARGET SDL2::SDL2)
      target_link_libraries(flat_sim_gui PUBLIC SDL2::SDL2)
    else()
      target_include_directories(flat_sim_gui PUBLIC ${SDL2_INCLUDE_DIRS})
      target_link_libraries(flat_sim_gui PUBLIC ${SDL2_LIBRARIES})
    endif()
  else()
    message(WARNING
      "flat_sim: 未找到 SDL2，仅构建 headless 版（GUI 功能关闭）。\n"
      "如需 GUI：sudo apt install libsdl2-dev 后重新运行 ./build.sh")
  endif()
endif()

# ---- ros：话题桥接（odom / base_scan / cmd_vel，无 TF）+ SLAM 视图订阅 ----
add_library(flat_sim_ros STATIC src/ros/RosBridge.cpp src/ros/SlamListener.cpp)
target_include_directories(flat_sim_ros PUBLIC ${FLAT_SIM_ROS_INCLUDES} ${Boost_INCLUDE_DIRS})
target_link_libraries(flat_sim_ros PUBLIC flat_sim_core ${FLAT_SIM_ROS_LIBS} ${Boost_LIBRARIES}
                                             Threads::Threads)

# ---- flat_sim_node ----
add_executable(flat_sim_node src/main.cpp)
target_link_libraries(flat_sim_node PRIVATE flat_sim_ros flat_sim_format flat_sim_core)
if(FLAT_SIM_HAVE_GUI)
  target_link_libraries(flat_sim_node PRIVATE flat_sim_gui)  # 同时带入 FLAT_SIM_HAVE_GUI 宏
endif()

# ---- 安装：install/bin/flat_sim_node + install/share/flat_sim/{worlds,models} ----
install(TARGETS flat_sim_node RUNTIME DESTINATION bin)
install(DIRECTORY worlds/ DESTINATION share/flat_sim/worlds)
install(DIRECTORY models/ DESTINATION share/flat_sim/models)
