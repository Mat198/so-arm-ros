cmake_minimum_required(VERSION 3.8)
project(so_arm_bring_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(so_arm_driver REQUIRED)
find_package(so_arm_config REQUIRED)
find_package(so_arm_hardware_interface REQUIRED)
find_package(so_arm_description REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
