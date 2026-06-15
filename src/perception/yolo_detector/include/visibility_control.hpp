#ifndef VISIBILITY_CONTROL_HPP
#define VISIBILITY_CONTROL_HPP

// DLL Export/Import Macros for Cross-Platform Component Library

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PERCEPTION_YOLO_EXPORT __attribute__((dllexport))
    #define PERCEPTION_YOLO_IMPORT __attribute__((dllimport))
  #else
    #define PERCEPTION_YOLO_EXPORT __declspec(dllexport)
    #define PERCEPTION_YOLO_IMPORT __declspec(dllimport)
  #endif
  #ifdef PERCEPTION_YOLO_BUILDING_DLL
    #define PERCEPTION_YOLO_PUBLIC PERCEPTION_YOLO_EXPORT
  #else
    #define PERCEPTION_YOLO_PUBLIC PERCEPTION_YOLO_IMPORT
  #endif
  #define PERCEPTION_YOLO_LOCAL
#else
  #define PERCEPTION_YOLO_EXPORT __attribute__((visibility("default")))
  #define PERCEPTION_YOLO_IMPORT
  #if __GNUC__ >= 4
    #define PERCEPTION_YOLO_PUBLIC __attribute__((visibility("default")))
    #define PERCEPTION_YOLO_LOCAL __attribute__((visibility("hidden")))
  #else
    #define PERCEPTION_YOLO_PUBLIC
    #define PERCEPTION_YOLO_LOCAL
  #endif
#endif

#endif // VISIBILITY_CONTROL_HPP