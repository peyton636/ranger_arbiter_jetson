cmake_minimum_required(VERSION 3.16)
project(yolo_detector VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Enable warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable testing and GPU support options
option(BUILD_TESTING "Build tests" ON)
option(ENABLE_GPU "Enable CUDA GPU support" OFF)

# Find ROS2 and other dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ===== Local YOLOs-CPP / ONNX Runtime (no download) =====
set(YOLOS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/YOLOs-CPP"
    CACHE PATH "Path to local YOLOs-CPP source")
set(ONNXRUNTIME_ROOT "${YOLOS_CPP_DIR}/thirdparty/onnxruntime"
    CACHE PATH "Path to ONNX Runtime root (under YOLOs-CPP/thirdparty)")

message(STATUS "YOLOs-CPP Source: ${YOLOS_CPP_DIR}")
message(STATUS "ONNX Runtime Root: ${ONNXRUNTIME_ROOT}")

# 在 thirdparty 下查找头文件和库文件
find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS ${ONNXRUNTIME_ROOT}
  PATH_SUFFIXES include onnxruntime/include
  NO_DEFAULT_PATH
)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  PATHS ${ONNXRUNTIME_ROOT}
  PATH_SUFFIXES lib onnxruntime/lib
  NO_DEFAULT_PATH
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR
    "ONNX Runtime not found in ${ONNXRUNTIME_ROOT}. "
    "Please check thirdparty layout."
  )
endif()

message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime lib: ${ONNXRUNTIME_LIB}")

# Include Directories
include_directories(
  include
  ${YOLOS_CPP_DIR}/include
  ${ONNXRUNTIME_INCLUDE_DIR}
)

# Component Library
add_library(${PROJECT_NAME}_components SHARED
  # Adapters
  src/adapters/detector_adapter.cpp
  src/adapters/segmentor_adapter.cpp
  src/adapters/pose_adapter.cpp
  src/adapters/obb_adapter.cpp
  src/adapters/classifier_adapter.cpp
  # Message Converters
  src/conversion/detection_converter.cpp
  src/conversion/segmentation_converter.cpp
  src/conversion/pose_converter.cpp
  src/conversion/obb_converter.cpp
  src/conversion/classification_converter.cpp
  # Nodes
  src/nodes/detector_node.cpp
  src/nodes/segmentor_node.cpp
  src/nodes/pose_node.cpp
  src/nodes/obb_node.cpp
  src/nodes/classifier_node.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  vision_msgs
  std_msgs
  geometry_msgs
  lifecycle_msgs
)

target_link_libraries(${PROJECT_NAME}_components
  ${ONNXRUNTIME_LIB}
  ${OpenCV_LIBS}
)

# Register Components
#　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝=================================
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "perception::YolosDetectorNode"
  EXECUTABLE yolos_detector_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "perception::YolosSegmentorNode"
  EXECUTABLE yolos_segmentor_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "perception::YolosPoseNode"
  EXECUTABLE yolos_pose_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "perception::YolosOBBNode"
  EXECUTABLE yolos_obb_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "perception::YolosClassifierNode"
  EXECUTABLE yolos_classifier_node
)
#　＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝==================================

# Set RPATH for ONNX Runtime
if(UNIX AND NOT APPLE)
  get_filename_component(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_LIB} DIRECTORY)
  set_target_properties(${PROJECT_NAME}_components PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_LIB_DIR}"
    INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}"
  )
endif()

# Installation
install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  
  # Unit tests (always run)
  ament_add_gtest(test_detection_converter
    test/unit/test_detection_converter.cpp
  )
  target_link_libraries(test_detection_converter ${PROJECT_NAME}_components)
  ament_target_dependencies(test_detection_converter rclcpp vision_msgs)
  
  # Integration tests (require model files via environment variables)
  # Set YOLOS_TEST_MODEL, YOLOS_TEST_LABELS, YOLOS_TEST_IMAGE to run
  
  macro(add_integration_test target_name source_file)
    ament_add_gtest(${target_name} ${source_file})
    target_link_libraries(${target_name} 
      ${PROJECT_NAME}_components 
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(${target_name} rclcpp rclcpp_lifecycle)
  endmacro()

  add_integration_test(test_integration_detection test/integration/test_detection.cpp)
  add_integration_test(test_integration_segmentation test/integration/test_segmentation.cpp)
  add_integration_test(test_integration_pose test/integration/test_pose.cpp)
  add_integration_test(test_integration_obb test/integration/test_obb.cpp)
  add_integration_test(test_integration_classification test/integration/test_classification.cpp)
endif()

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_components)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  vision_msgs
)

ament_package()

# Summary
message(STATUS "")
message(STATUS "=== ros2_yolos_cpp Configuration ===")
message(STATUS "  Version:       ${PROJECT_VERSION}")
message(STATUS "  Build Type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenCV:        ${OpenCV_VERSION}")
message(STATUS "  ONNX Runtime:  ${ONNXRUNTIME_LIB}")
message(STATUS "  YOLOs-CPP:     ${YOLOS_CPP_DIR}")
message(STATUS "  GPU Support:   ${ENABLE_GPU}")
message(STATUS "")

