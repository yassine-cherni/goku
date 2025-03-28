cmake_minimum_required(VERSION 3.5)
project(diff_drive_hw_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/diff_drive_hw_interface.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp)

pluginlib_export_plugin_description_file(hardware_interface diff_drive_hw_interface.xml)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES diff_drive_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
