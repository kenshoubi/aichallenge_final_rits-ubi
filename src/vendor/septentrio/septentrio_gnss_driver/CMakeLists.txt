cmake_minimum_required(VERSION 3.5)
project(septentrio_gnss_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/septentrio_gnss_driver/node/rosaic_node.cpp
  src/septentrio_gnss_driver/communication/circular_buffer.cpp
  src/septentrio_gnss_driver/parsers/parsing_utilities.cpp
  src/septentrio_gnss_driver/parsers/string_utilities.cpp
  src/septentrio_gnss_driver/parsers/nmea_parsers/gpgga.cpp
  src/septentrio_gnss_driver/parsers/nmea_parsers/gprmc.cpp
  src/septentrio_gnss_driver/parsers/nmea_parsers/gpgsa.cpp
  src/septentrio_gnss_driver/parsers/nmea_parsers/gpgsv.cpp
  src/septentrio_gnss_driver/crc/crc.c
  src/septentrio_gnss_driver/communication/communication_core.cpp
  src/septentrio_gnss_driver/communication/rx_message.cpp
  src/septentrio_gnss_driver/communication/callback_handlers.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)

