cmake_minimum_required(VERSION 3.8)
project(object_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(object_localization_node src/object_localization_node.cpp)
ament_target_dependencies(object_localization_node rclcpp)

install(TARGETS
  object_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
