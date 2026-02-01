cmake_minimum_required(VERSION 3.8)
project(so_arm_rviz_tcp_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(so_arm_driver REQUIRED)

set(PKG_INCLUDES
  rclcpp
  pluginlib
  rviz_common
  std_srvs
  so_arm_driver
)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/tcp_plugin.hpp
  include/${PROJECT_NAME}/pose_tree_widget.hpp
  include/${PROJECT_NAME}/pose_delegate.hpp
)

add_library(so_arm_tcp_panel src/tcp_plugin.cpp src/pose_tree_widget.cpp ${MOC_FILES})
target_include_directories(so_arm_tcp_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(so_arm_tcp_panel ${PKG_INCLUDES})

install(TARGETS so_arm_tcp_panel
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(FILES so_arm_tcp_plugin.xml
        DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(rviz_common so_arm_tcp_plugin.xml)


install(DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
