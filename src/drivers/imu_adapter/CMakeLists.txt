cmake_minimum_required(VERSION 3.8)
project(imu_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jetson_mcu_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/imu_adapter_node.cpp
  src/serial/serial_port.cpp
  src/wit/wit_protocol.cpp
  src/jetson/mcu_virtual_time_sync.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  geometry_msgs
  jetson_mcu_msgs
  lifecycle_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "imu_adapter::ImuAdapterNode"
  EXECUTABLE imu_adapter_node
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
  sensor_msgs
  geometry_msgs
  jetson_mcu_msgs
  lifecycle_msgs
)

ament_package()
