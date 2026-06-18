cmake_minimum_required(VERSION 3.8)
project(agx_gripper_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(agx_arm_msgs REQUIRED)
find_package(agx_motion_msgs REQUIRED)

add_executable(gripper_controller_node src/gripper_controller_node.cpp)
ament_target_dependencies(gripper_controller_node
  rclcpp
  sensor_msgs
  agx_arm_msgs
  agx_motion_msgs)

install(TARGETS gripper_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(FILES GRIPPER_CONTROLLER_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
