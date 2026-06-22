cmake_minimum_required(VERSION 3.8)
project(target_tracker)

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
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/tracker/pose_filter.cpp
  src/tracker/track_manager.cpp
  src/target_tracker_node.cpp
)

target_compile_definitions(${PROJECT_NAME}_components PRIVATE TARGET_TRACKER_BUILDING_DLL)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  visualization_msgs
  lifecycle_msgs
  std_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "perception::TargetTrackerNode"
  EXECUTABLE target_tracker_node
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
  visualization_msgs
  lifecycle_msgs
)

ament_package()
