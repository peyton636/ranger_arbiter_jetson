cmake_minimum_required(VERSION 3.8)
project(agx_gripper_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(agx_arm_msgs REQUIRED)
find_package(agx_motion_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_components SHARED
  src/gripper_controller_node.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Register Components
#　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝=================================
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "manipulation::GripperControllerNode"
  EXECUTABLE gripper_controller_node
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  sensor_msgs
  agx_arm_msgs
  agx_motion_msgs
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
)

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES GRIPPER_CONTROLLER_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_components)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  agx_arm_msgs
  agx_motion_msgs
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  ament_index_cpp
)

ament_package()
