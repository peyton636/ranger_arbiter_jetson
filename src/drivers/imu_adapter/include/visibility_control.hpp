#ifndef IMU_ADAPTER__VISIBILITY_CONTROL_HPP_
#define IMU_ADAPTER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMU_ADAPTER_EXPORT __attribute__((dllexport))
    #define IMU_ADAPTER_IMPORT __attribute__((dllimport))
  #else
    #define IMU_ADAPTER_EXPORT __declspec(dllexport)
    #define IMU_ADAPTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMU_ADAPTER_BUILDING_DLL
    #define IMU_ADAPTER_PUBLIC IMU_ADAPTER_EXPORT
  #else
    #define IMU_ADAPTER_PUBLIC IMU_ADAPTER_IMPORT
  #endif
  #define IMU_ADAPTER_LOCAL
#else
  #define IMU_ADAPTER_EXPORT __attribute__((visibility("default")))
  #define IMU_ADAPTER_IMPORT
  #if __GNUC__ >= 4
    #define IMU_ADAPTER_PUBLIC __attribute__((visibility("default")))
    #define IMU_ADAPTER_LOCAL __attribute__((visibility("hidden")))
  #else
    #define IMU_ADAPTER_PUBLIC
    #define IMU_ADAPTER_LOCAL
  #endif
#endif

#endif  // IMU_ADAPTER__VISIBILITY_CONTROL_HPP_
