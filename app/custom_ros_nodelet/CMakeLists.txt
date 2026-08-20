# custom_ros_nodelet：JSON 驱动的 Nodelet manager（独立 CMake，无 package.xml）。
# 由本目录 ./make.sh 在已有 custom_mini_install（ROS 环境）之上编译。
# nlohmann/json.hpp 放在本包 include/ 下。
cmake_minimum_required(VERSION 3.0.2)
project(custom_ros_nodelet)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(roscpp REQUIRED)
find_package(nodelet REQUIRED)
find_package(class_loader REQUIRED)

add_executable(custom_ros_nodelet src/custom_ros_nodelet.cpp)
target_include_directories(custom_ros_nodelet PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${roscpp_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
)
target_link_libraries(custom_ros_nodelet
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${class_loader_LIBRARIES}
)

install(TARGETS custom_ros_nodelet
  RUNTIME DESTINATION bin
)
