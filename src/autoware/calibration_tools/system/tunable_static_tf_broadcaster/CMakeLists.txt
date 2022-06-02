cmake_minimum_required(VERSION 3.5)
project(tunable_static_tf_broadcaster)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Targets
ament_auto_add_library(tunable_static_tf_broadcaster_node_component SHARED
  src/tunable_static_tf_broadcaster_node.cpp
)

rclcpp_components_register_node(tunable_static_tf_broadcaster_node_component
  PLUGIN "tunable_static_tf_broadcaster::TunableStaticTfBroadcasterNode"
  EXECUTABLE tunable_static_tf_broadcaster_node
)

## Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    params
    launch
)
