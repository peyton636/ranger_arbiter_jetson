#ifndef VISIBILITY_CONTROL_HPP
#define VISIBILITY_CONTROL_HPP

// DLL Export/Import Macros for Cross-Platform Component Library

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PERCEPTION_OBJ_LOCALIZATION_EXPORT __attribute__((dllexport))
    #define PERCEPTION_OBJ_LOCALIZATION_IMPORT __attribute__((dllimport))
  #else
    #define PERCEPTION_OBJ_LOCALIZATION_EXPORT __declspec(dllexport)
    #define PERCEPTION_OBJ_LOCALIZATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef PERCEPTION_OBJ_LOCALIZATION_BUILDING_DLL
    #define PERCEPTION_OBJ_LOCALIZATION_PUBLIC PERCEPTION_OBJ_LOCALIZATION_EXPORT
  #else
    #define PERCEPTION_OBJ_LOCALIZATION_PUBLIC PERCEPTION_OBJ_LOCALIZATION_IMPORT
  #endif
  #define PERCEPTION_OBJ_LOCALIZATION_LOCAL
#else
  #define PERCEPTION_OBJ_LOCALIZATION_EXPORT __attribute__((visibility("default")))
  #define PERCEPTION_OBJ_LOCALIZATION_IMPORT
  #if __GNUC__ >= 4
    #define PERCEPTION_OBJ_LOCALIZATION_PUBLIC __attribute__((visibility("default")))
    #define PERCEPTION_OBJ_LOCALIZATION_LOCAL __attribute__((visibility("hidden")))
  #else
    #define PERCEPTION_OBJ_LOCALIZATION_PUBLIC
    #define PERCEPTION_OBJ_LOCALIZATION_LOCAL
  #endif
#endif

#endif // VISIBILITY_CONTROL_HPP