# heartbeat_nodelet：心跳 Nodelet → libheartbeat_nodelet.so
# 独立 CMake（无 catkin / 不依赖 package.xml），由 ./make.sh 在 custom_mini_install 上编译。
cmake_minimum_required(VERSION 3.0.2)
project(heartbeat_nodelet)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(roscpp REQUIRED)
find_package(nodelet REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

add_library(heartbeat_nodelet SHARED src/heartbeat_nodelet.cpp)
target_include_directories(heartbeat_nodelet PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../common_include
  ${roscpp_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)
target_link_libraries(heartbeat_nodelet
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

install(TARGETS heartbeat_nodelet LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/nodelet_plugins.xml")
  install(FILES nodelet_plugins.xml DESTINATION share/heartbeat_nodelet)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
  install(FILES package.xml DESTINATION share/heartbeat_nodelet)
endif()
