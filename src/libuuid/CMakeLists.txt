# 本仓库自己写的最小 libuuid，安装到 CMAKE_INSTALL_PREFIX（即项目 install/）。
cmake_minimum_required(VERSION 3.0)
project(uuid C)

add_library(uuid SHARED uuid.c)
target_include_directories(uuid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(uuid PROPERTIES
  VERSION 1.0.0
  SOVERSION 1
  PUBLIC_HEADER include/uuid/uuid.h)

install(TARGETS uuid
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  PUBLIC_HEADER DESTINATION include/uuid)
install(DIRECTORY include/uuid DESTINATION include)
