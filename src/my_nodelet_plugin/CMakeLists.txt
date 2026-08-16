# my_nodelet_plugin：隔离环境里的演示用 Nodelet 插件。
# 产出共享库 libmy_nodelet_plugin.so，供 nodelet 管理器动态加载。
cmake_minimum_required(VERSION 3.0.2)
project(my_nodelet_plugin)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_nodelet_plugin
  CATKIN_DEPENDS nodelet pluginlib roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(my_nodelet_plugin
  src/talker_nodelet.cpp
  src/listener_nodelet.cpp
)
target_link_libraries(my_nodelet_plugin ${catkin_LIBRARIES})

install(TARGETS my_nodelet_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 必须安装插件描述，否则 rosrun nodelet load 找不到类名。
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
