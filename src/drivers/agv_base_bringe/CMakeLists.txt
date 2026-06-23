cmake_minimum_required(VERSION 3.8)
project(agv_base_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(jetson_mcu_msgs REQUIRED)
find_package(scr_sensor REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/agv_base_bringe_node.cpp
  src/jetson/jetson_protocol.cpp
  src/jetson/mcu_time_sync.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  jetson_mcu_msgs
  scr_sensor
  lifecycle_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "agv_base_driver::AgvBaseBringeNode"
  EXECUTABLE agv_base_bringe_node
)

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_components)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  jetson_mcu_msgs
  scr_sensor
  lifecycle_msgs
)

ament_package()
