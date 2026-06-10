# cmake_minimum_required(VERSION 3.8)
# project(scr_sensor)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# # Generate messages in the 'msg' folder
# FILE(GLOB_RECURSE msg_files "*.msg")
# FILE(GLOB_RECURSE srv_files "*.srv")
# function(get_relative_path var)
#   set(relative_paths "")
#   foreach(path ${${var}})
#     file(RELATIVE_PATH rel_path ${PROJECT_SOURCE_DIR} ${path})
#     list(APPEND relative_paths ${rel_path})
#   endforeach()
#   set(${var} "${relative_paths}" PARENT_SCOPE)
# endfunction()

# get_relative_path(msg_files)
# get_relative_path(srv_files)

# rosidl_generate_interfaces(
#     ${PROJECT_NAME}
#     ${msg_files}
#     ${srv_files}
#     DEPENDENCIES
#     sensor_msgs
#     std_msgs
# )

# if(BUILD_TESTING)
#     find_package(ament_lint_auto REQUIRED)
#     # the following line skips the linter which checks for copyrights
#     # uncomment the line when a copyright and license is not present in all source files
#     #set(ament_cmake_copyright_FOUND TRUE)
#     # the following line skips cpplint (only works in a git repo)
#     # uncomment the line when this package is not in a git repo
#     #set(ament_cmake_cpplint_FOUND TRUE)
#     ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
