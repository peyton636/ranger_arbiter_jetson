cmake_minimum_required(VERSION 3.10)
project(image_preprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_preprocess_node
  src/main.cpp
  src/preprocess_node.cpp
)

ament_target_dependencies(image_preprocess_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(image_preprocess_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

install(TARGETS image_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()