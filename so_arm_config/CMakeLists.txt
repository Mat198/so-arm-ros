cmake_minimum_required(VERSION 3.22)
project(so_arm_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
