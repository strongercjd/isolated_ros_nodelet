# map_log_nodelet：SLAM 建图日志记录 → libmap_log_nodelet.so
# 独立 CMake（无 catkin / 不依赖 package.xml），由 ./make.sh 在 custom_mini_install 上编译。
cmake_minimum_required(VERSION 3.0.2)
project(map_log_nodelet)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(roscpp REQUIRED)
find_package(nodelet REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag REQUIRED)
find_package(rosbag_storage REQUIRED)

add_library(map_log_nodelet SHARED src/map_log_nodelet.cpp)
target_include_directories(map_log_nodelet PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../common_include
  ${roscpp_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rosbag_INCLUDE_DIRS}
  ${rosbag_storage_INCLUDE_DIRS}
)
target_link_libraries(map_log_nodelet
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${rosbag_LIBRARIES}
  ${rosbag_storage_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

install(TARGETS map_log_nodelet LIBRARY DESTINATION lib)
