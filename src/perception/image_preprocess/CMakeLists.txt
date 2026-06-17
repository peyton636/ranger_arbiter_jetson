cmake_minimum_required(VERSION 3.16)
project(image_preprocess VERSION 1.0.0 LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED) 
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_components SHARED
  src/preprocess_node.cpp
)

target_include_directories(${PROJECT_NAME}_components
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  rclcpp_lifecycle 
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(${PROJECT_NAME}_components
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

# Register Components
rclcpp_components_register_nodes(${PROJECT_NAME}_components
  PLUGIN "perception::PreprocessNode"
  EXECUTABLE preprocess_node
)

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_components)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  rclcpp_lifecycle
)

ament_package()