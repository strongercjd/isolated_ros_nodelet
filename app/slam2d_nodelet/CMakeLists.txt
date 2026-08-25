# slam2d_nodelet：2D 激光 SLAM Nodelet → libslam2d_nodelet.so
# 独立 CMake（无 catkin / 不依赖 package.xml），由 ./make.sh 在 custom_mini_install 上编译。
#
# 算法源码 vendoring 自 lidarslam_2d（src/slam2d/{slam,unity}，已剥离
# cv_ui/OpenCV、Ceres、rosbag、tf 依赖），Eigen 头文件来自
# custom_mini_install/include/eigen3（custom_mini.sh 的 setup_eigen 步骤提供）。
cmake_minimum_required(VERSION 3.0.2)
project(slam2d_nodelet)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(roscpp REQUIRED)
find_package(nodelet REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 移植的算法源（算法逻辑与原工程一致，仅剥离显示接口）
set(SLAM2D_ALGO_SRC
  src/slam2d/unity/transform.cpp
  src/slam2d/unity/grid_map.cpp
  src/slam2d/unity/voxel_filter.cpp
  src/slam2d/slam/front_end_process.cpp
  src/slam2d/slam/laser_odometry.cpp
  src/slam2d/slam/icp2d.cpp
  src/slam2d/slam/point_to_point.cpp
  src/slam2d/slam/laser_mapping.cpp
)

add_library(slam2d_nodelet SHARED
  src/slam2d_nodelet.cpp
  ${SLAM2D_ALGO_SRC}
)

# src/slam2d 进 include 路径后，算法源码里的 "unity/xxx.h" / "slam/xxx.h" 原样可解析；
# src 供节点层引用 "slam_input.h"。
target_include_directories(slam2d_nodelet PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/slam2d
  ${CMAKE_CURRENT_SOURCE_DIR}/../common_include
  ${roscpp_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

target_link_libraries(slam2d_nodelet
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)
# Eigen 纯头文件无需链接；<eigen3/Eigen/...> 由 make.sh 的 -I${ROS_INSTALL}/include 解析

install(TARGETS slam2d_nodelet LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
# package.xml / nodelet_plugins.xml 仅供官方路径 pluginlib 发现；本仓库走 plugins.json，缺失不影响编译。
