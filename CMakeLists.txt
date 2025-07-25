cmake_minimum_required(VERSION 3.5)
project(microstrain_mv5_can_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(can_dbc_parser REQUIRED)
find_package(j1939_msgs)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/microstrain_mv5_can_driver.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-function)

rclcpp_components_register_node(microstrain_mv5_can_driver
  PLUGIN "ros2_j1939::MicrostrainMV5CanDriver"
  EXECUTABLE microstrain_mv5_can_driver_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)