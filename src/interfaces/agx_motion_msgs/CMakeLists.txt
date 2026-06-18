cmake_minimum_required(VERSION 3.8)
project(agx_motion_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanRequest.msg"
  "msg/MotionTrajectory.msg"
  "msg/ExecuteFeedback.msg"
  "msg/ExecuteResult.msg"
  "msg/GripperCmd.msg"
  "msg/GripperControlStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs moveit_msgs
)

ament_package()
