# fastbuild_task_nodelet：快建任务节点 → libfastbuild_task_nodelet.so
# 独立 CMake（无 catkin / 不依赖 package.xml），由 ./make.sh 在 custom_mini_install 上编译。
cmake_minimum_required(VERSION 3.0.2)
project(fastbuild_task_nodelet)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(roscpp REQUIRED)
find_package(nodelet REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(fastbuild_task_nodelet SHARED
  src/fastbuild_task_nodelet.cpp
)
target_include_directories(fastbuild_task_nodelet PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../common_include
  ${roscpp_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)
target_link_libraries(fastbuild_task_nodelet
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

install(TARGETS fastbuild_task_nodelet LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
