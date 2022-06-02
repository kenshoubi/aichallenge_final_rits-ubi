cmake_minimum_required(VERSION 3.5)
project(septentrio_gnss_driver_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BlockHeader.msg"
  "msg/PVTCartesian.msg"
  "msg/PVTGeodetic.msg"
  "msg/PosCovCartesian.msg"
  "msg/PosCovGeodetic.msg"
  "msg/AttEuler.msg"
  "msg/AttCovEuler.msg"
  "msg/Gpgga.msg"
  "msg/Gprmc.msg"
  "msg/Gpgsa.msg"
  "msg/Gpgsv.msg"
  "msg/GpgsvSatellite.msg"
  DEPENDENCIES
    sensor_msgs
    std_msgs
    diagnostic_msgs
    gps_msgs
)

ament_package()

