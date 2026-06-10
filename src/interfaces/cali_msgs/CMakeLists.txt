cmake_minimum_required(VERSION 3.8)
project(cali_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate messages in the 'msg' folder
FILE(GLOB_RECURSE msg_files "*.msg")
function(get_relative_path var)
set(relative_paths "")
foreach(path ${${var}})
  file(RELATIVE_PATH rel_path ${PROJECT_SOURCE_DIR} ${path})
  list(APPEND relative_paths ${rel_path})
endforeach()
set(${var} "${relative_paths}" PARENT_SCOPE)
endfunction()

get_relative_path(msg_files)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  )
ament_package()
