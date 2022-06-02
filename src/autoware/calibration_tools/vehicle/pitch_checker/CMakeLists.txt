cmake_minimum_required(VERSION 3.5)
project(pitch_checker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(pitch_checker
  src/pitch_checker_node.cpp
  src/main.cpp)
ament_target_dependencies(pitch_checker)

ament_auto_add_library(pitch_compare
  src/pitch_compare.cpp
  src/pitch_reader.cpp)
ament_target_dependencies(pitch_compare)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS scripts/view_pitch.py DESTINATION lib/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config)
