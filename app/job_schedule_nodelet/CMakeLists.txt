# job_schedule_nodelet：任务调度节点 → libjob_schedule_nodelet.so
# 独立 CMake（无 catkin / 不依赖 package.xml），由 ./make.sh 在 custom_mini_install 上编译。
cmake_minimum_required(VERSION 3.0.2)
project(job_schedule_nodelet)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(roscpp REQUIRED)
find_package(nodelet REQUIRED)
find_package(pluginlib REQUIRED)

add_library(job_schedule_nodelet SHARED
  src/job_nodelet.cpp
)
target_include_directories(job_schedule_nodelet PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../common_include
  ${roscpp_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
)
target_link_libraries(job_schedule_nodelet
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${pluginlib_LIBRARIES}
)

install(TARGETS job_schedule_nodelet LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
