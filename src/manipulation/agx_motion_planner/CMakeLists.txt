cmake_minimum_required(VERSION 3.8)
project(agx_motion_planner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(agx_motion_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(motion_planner_node src/motion_planner_node.cpp)
target_include_directories(motion_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(motion_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  agx_motion_msgs
  tf2_geometry_msgs)

install(TARGETS motion_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(FILES MOTION_PLANNER_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
