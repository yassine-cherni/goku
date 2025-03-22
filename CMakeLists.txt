cmake_minimum_required(VERSION 3.5)
project(goku)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)
find_package(robot_localization REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_bringup REQUIRED)

add_library(diffbot_system_hardware SHARED src/diffbot_system.cpp)
target_include_directories(diffbot_system_hardware PUBLIC include)
ament_target_dependencies(diffbot_system_hardware 
  rclcpp hardware_interface pluginlib serial robot_localization)

pluginlib_export_plugin_description_file(hardware_interface diffbot_system.xml)

install(TARGETS diffbot_system_hardware
  DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch config urdf ros2_control
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
