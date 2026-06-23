#ifndef AGV_BASE_DRIVER__VISIBILITY_CONTROL_HPP_
#define AGV_BASE_DRIVER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AGV_BASE_DRIVER_EXPORT __attribute__((dllexport))
    #define AGV_BASE_DRIVER_IMPORT __attribute__((dllimport))
  #else
    #define AGV_BASE_DRIVER_EXPORT __declspec(dllexport)
    #define AGV_BASE_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef AGV_BASE_DRIVER_BUILDING_DLL
    #define AGV_BASE_DRIVER_PUBLIC AGV_BASE_DRIVER_EXPORT
  #else
    #define AGV_BASE_DRIVER_PUBLIC AGV_BASE_DRIVER_IMPORT
  #endif
  #define AGV_BASE_DRIVER_LOCAL
#else
  #define AGV_BASE_DRIVER_EXPORT __attribute__((visibility("default")))
  #define AGV_BASE_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define AGV_BASE_DRIVER_PUBLIC __attribute__((visibility("default")))
    #define AGV_BASE_DRIVER_LOCAL __attribute__((visibility("hidden")))
  #else
    #define AGV_BASE_DRIVER_PUBLIC
    #define AGV_BASE_DRIVER_LOCAL
  #endif
#endif

#endif  // AGV_BASE_DRIVER__VISIBILITY_CONTROL_HPP_
