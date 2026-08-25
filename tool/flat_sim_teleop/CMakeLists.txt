# =============================================================================
# flat_sim_teleop —— flat_sim 键盘遥控（C++ / CMake）
# 对外入口：tool/flat_sim_teleop/build.sh
# =============================================================================
cmake_minimum_required(VERSION 3.10)
project(flat_sim_teleop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(NOT DEFINED Boost_NO_SYSTEM_PATHS)
  set(Boost_NO_SYSTEM_PATHS ON)
endif()
find_package(Boost REQUIRED COMPONENTS system thread chrono)
find_package(Threads REQUIRED)

add_executable(flat_sim_teleop src/main.cpp)
target_include_directories(flat_sim_teleop PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${roscpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})
target_link_libraries(flat_sim_teleop PRIVATE
  ${roscpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${Boost_LIBRARIES}
  Threads::Threads)

install(TARGETS flat_sim_teleop RUNTIME DESTINATION bin)
