# ============================================================================
# YOLOs-CPP - High-Performance C++ Inference for YOLO Models
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(YOLOs-CPP VERSION 2.0.0 LANGUAGES CXX)

# ============================================================================
# C++ Standard
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Build Options
# ============================================================================
option(BUILD_EXAMPLES "Build example applications" OFF)
option(ENABLE_WARNINGS "Enable compiler warnings" ON)

# ============================================================================
# ONNX Runtime
# ============================================================================
set(ONNXRUNTIME_DIR "${CMAKE_SOURCE_DIR}/thirdparty/onnxruntime")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_DIR}/include")
set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")

if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "ONNX Runtime headers not found: ${ONNXRUNTIME_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${ONNXRUNTIME_LIB}")
  message(FATAL_ERROR "ONNX Runtime library not found: ${ONNXRUNTIME_LIB}")
endif()

message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime lib: ${ONNXRUNTIME_LIB}")

message(STATUS "ONNX Runtime: ${ONNXRUNTIME_DIR}")

# ============================================================================
# OpenCV
# ============================================================================
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV: ${OpenCV_VERSION}")

# ============================================================================
# Include Directories
# ============================================================================
include_directories(
    "${CMAKE_SOURCE_DIR}/include"
    "${ONNXRUNTIME_DIR}/include"
)

# ============================================================================
# Compiler Flags
# ============================================================================
if(ENABLE_WARNINGS)
    if(MSVC)
        add_compile_options(/W4)
    else()
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
endif()

# Release optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(NOT MSVC)
        add_compile_options(-O3)
    endif()
endif()

# Threading support
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ============================================================================
# ONNX Runtime Library
# ============================================================================
if(UNIX AND NOT APPLE)
    set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
    message(STATUS "Platform: Linux")
elseif(APPLE)
    set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/libonnxruntime.dylib")
    message(STATUS "Platform: macOS")
elseif(WIN32)
    set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/onnxruntime.lib")
    message(STATUS "Platform: Windows")
endif()

# ============================================================================
# Helper Function
# ============================================================================
function(add_yolo_executable name source)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE "${ONNXRUNTIME_DIR}/include")
    target_link_libraries(${name} PRIVATE 
        ${OpenCV_LIBS}
        ${ONNXRUNTIME_LIB}
        Threads::Threads
    )
    
    # Platform-specific runtime library handling
    if(WIN32)
        # Copy ONNX Runtime DLL to output directory
        set(ONNXRUNTIME_DLL "${ONNXRUNTIME_DIR}/lib/onnxruntime.dll")
        if(EXISTS "${ONNXRUNTIME_DLL}")
            add_custom_command(TARGET ${name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${ONNXRUNTIME_DLL}"
                "$<TARGET_FILE_DIR:${name}>"
            )
        endif()
    elseif(UNIX AND NOT APPLE)
        # Set RPATH for runtime library discovery
        set_target_properties(${name} PROPERTIES
            BUILD_RPATH "${ONNXRUNTIME_DIR}/lib"
            INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
        )
    elseif(APPLE)
        # Set RPATH for macOS
        set_target_properties(${name} PROPERTIES
            BUILD_RPATH "@executable_path;${ONNXRUNTIME_DIR}/lib"
            INSTALL_RPATH "@executable_path;${ONNXRUNTIME_DIR}/lib"
        )
    endif()
endfunction()

# ============================================================================
# Main Executables
# ============================================================================
add_yolo_executable(image_inference src/image_inference.cpp)
add_yolo_executable(video_inference src/video_inference.cpp)
add_yolo_executable(video_yolo26_all_tasks src/video_yolo26_all_tasks.cpp)
add_yolo_executable(video_yolo26_seg_pose src/video_yolo26_seg_pose.cpp)
add_yolo_executable(video_yoloe_seg src/video_yoloe_seg.cpp)
add_yolo_executable(image_yoloe_seg src/image_yoloe_seg.cpp)
add_yolo_executable(camera_inference src/camera_inference.cpp)
add_yolo_executable(class_image_inference src/class_image_inference.cpp)
add_yolo_executable(batch_image_inference src/batch_image_inference.cpp)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "=== YOLOs-CPP Configuration ===")
message(STATUS "  Version:       ${PROJECT_VERSION}")
message(STATUS "  Build Type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenCV:        ${OpenCV_VERSION}")
message(STATUS "  ONNX Runtime:  ${ONNXRUNTIME_DIR}")
message(STATUS "  Examples:      ${BUILD_EXAMPLES}")
message(STATUS "")
