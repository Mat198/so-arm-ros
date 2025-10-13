cmake_minimum_required(VERSION 3.8)
project(so_arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 base dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(so_arm_driver REQUIRED)

# Hardware interface
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

### Hardware interface ###

set(HARDWARE_INTERFACE_INCLUDES
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
  so_arm_driver

  hardware_interface
  pluginlib
)

add_library(so_arm_hardware_interface
  SHARED
  src/arm_interface.cpp
)

target_include_directories(
  so_arm_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  so_arm_hardware_interface
  ${HARDWARE_INTERFACE_INCLUDES}
)

ament_export_libraries(so_arm_hardware_interface)

install(
  TARGETS so_arm_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(hardware_interface so_arm_hardware_interface.xml)

### Install includes ###

install(
  DIRECTORY include
  DESTINATION include
)

ament_package()
