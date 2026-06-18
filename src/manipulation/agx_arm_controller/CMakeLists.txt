cmake_minimum_required(VERSION 3.8)
project(agx_arm_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(agx_motion_msgs REQUIRED)

add_executable(arm_controller_node src/arm_controller_node.cpp)
ament_target_dependencies(arm_controller_node
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  agx_motion_msgs)

install(TARGETS arm_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(FILES ARM_CONTROLLER_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
