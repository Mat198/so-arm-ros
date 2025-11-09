cmake_minimum_required(VERSION 3.8)
project(so_arm_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(so_arm_driver REQUIRED)

include_directories(
  include
)

set(PKG_INCLUDES
  rclcpp
  control_msgs
  controller_manager_msgs
  std_srvs
  so_arm_driver
)

add_executable(so_arm_manager src/manager.cpp src/main.cpp)
target_include_directories(so_arm_manager PRIVATE include)
ament_target_dependencies(so_arm_manager ${PKG_INCLUDES})

install(TARGETS so_arm_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
