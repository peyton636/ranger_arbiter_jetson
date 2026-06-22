cmake_minimum_required(VERSION 3.8)
project(task_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(agx_motion_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/types/task_types.cpp
  src/types/task_context.cpp
  src/policy/pick_place_policy.cpp
  src/clients/motion_client.cpp
  src/clients/gripper_client.cpp
  src/clients/navigation_client.cpp
  src/handlers/perception_handler.cpp
  src/fsm/state_machine.cpp
  src/task_fsm_node.cpp
)

target_compile_definitions(${PROJECT_NAME}_components PRIVATE TASK_FSM_BUILDING_DLL)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  agx_motion_msgs
  lifecycle_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "coordination::TaskFsmNode"
  EXECUTABLE task_fsm_node
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
  std_msgs
  std_srvs
  geometry_msgs
  agx_motion_msgs
)

ament_package()
