cmake_minimum_required(VERSION 3.5)
project(microstrain_mv5_can_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Export compile commands by default (helpful for clang-tidy and autocomplete for certain IDEs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "" FORCE)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(can_dbc_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/microstrain_mv5_can_driver_node.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-function)

rclcpp_components_register_node(microstrain_mv5_can_driver
  PLUGIN "drivers::microstrain::MicrostrainMV5CanDriverNode"
  EXECUTABLE microstrain_mv5_can_driver_exe
)

#run colcon test to run linters against code
if(BUILD_TESTING)
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)