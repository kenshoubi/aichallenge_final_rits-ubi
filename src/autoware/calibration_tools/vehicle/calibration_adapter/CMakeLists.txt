cmake_minimum_required(VERSION 3.5)
project(calibration_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(pacmod_calibration_adapter
  src/pacmod_calibration_adapter_node.cpp
  src/calibration_adapter_node_base.cpp)
ament_target_dependencies(pacmod_calibration_adapter)

ament_auto_add_executable(calibration_adapter
  src/calibration_adapter_node.cpp
  src/calibration_adapter_node_base.cpp)
ament_target_dependencies(calibration_adapter)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch)
