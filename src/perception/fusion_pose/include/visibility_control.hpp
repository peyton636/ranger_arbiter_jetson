#ifndef VISIBILITY_CONTROL_HPP
#define VISIBILITY_CONTROL_HPP

// DLL Export/Import Macros for Cross-Platform Component Library

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PERCEPTION_FUSION_POSE_EXPORT __attribute__((dllexport))
    #define PERCEPTION_FUSION_POSE_IMPORT __attribute__((dllimport))
  #else
    #define PERCEPTION_FUSION_POSE_EXPORT __declspec(dllexport)
    #define PERCEPTION_FUSION_POSE_IMPORT __declspec(dllimport)
  #endif
  #ifdef PERCEPTION_FUSION_POSE_BUILDING_DLL
    #define PERCEPTION_FUSION_POSE_PUBLIC PERCEPTION_FUSION_POSE_EXPORT
  #else
    #define PERCEPTION_FUSION_POSE_PUBLIC PERCEPTION_FUSION_POSE_IMPORT
  #endif
  #define PERCEPTION_FUSION_POSE_LOCAL
#else
  #define PERCEPTION_FUSION_POSE_EXPORT __attribute__((visibility("default")))
  #define PERCEPTION_FUSION_POSE_IMPORT
  #if __GNUC__ >= 4
    #define PERCEPTION_FUSION_POSE_PUBLIC __attribute__((visibility("default")))
    #define PERCEPTION_FUSION_POSE_LOCAL __attribute__((visibility("hidden")))
  #else
    #define PERCEPTION_FUSION_POSE_PUBLIC
    #define PERCEPTION_FUSION_POSE_LOCAL
  #endif
#endif

#endif // VISIBILITY_CONTROL_HPP