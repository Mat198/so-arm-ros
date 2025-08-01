cmake_minimum_required(VERSION 3.8)
project(so_arm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sc_servo REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(so_arm_driver src/so_arm_driver.cpp)
target_compile_features(so_arm_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(so_arm_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  so_arm_driver
    rclcpp
    sc_servo
)

add_executable(so_arm_demo test/so_arm_driver_test_node.cpp src/so_arm_driver.cpp)
target_include_directories(so_arm_demo PRIVATE include)
ament_target_dependencies(so_arm_demo rclcpp sc_servo trajectory_msgs sensor_msgs)

add_executable(so_arm_keyboard_controller test/keyboard_controller.cpp)
target_include_directories(so_arm_keyboard_controller PRIVATE include)
ament_target_dependencies(so_arm_keyboard_controller rclcpp trajectory_msgs sensor_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS so_arm_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS so_arm_demo so_arm_keyboard_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  so_arm_driver
)
ament_export_dependencies(rclcpp sc_servo)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
