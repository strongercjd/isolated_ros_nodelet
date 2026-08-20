# 独立 CMake 工程（无 package.xml），不会被 catkin 扫到。
# 由 custom_mini.sh 在编完 custom_mini_install 里的 ROS 栈之后单独编译。
cmake_minimum_required(VERSION 3.0.2)
project(custom_mini)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(roscpp REQUIRED)
find_package(nodelet REQUIRED)
find_package(class_loader REQUIRED)

set(NLOHMANN_JSON_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../nlohmann_json/include")
if(NOT EXISTS "${NLOHMANN_JSON_INCLUDE}/nlohmann/json.hpp")
  message(FATAL_ERROR "缺少 nlohmann/json.hpp，请先运行 ./custom_mini.sh build（会自动下载）")
endif()

add_executable(custom_mini_manager src/custom_mini_manager.cpp)
target_include_directories(custom_mini_manager PRIVATE
  ${roscpp_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
  ${NLOHMANN_JSON_INCLUDE}
)
target_link_libraries(custom_mini_manager
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${class_loader_LIBRARIES}
)

install(TARGETS custom_mini_manager
  RUNTIME DESTINATION bin
)
