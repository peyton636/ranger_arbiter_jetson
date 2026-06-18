#ifndef VISIBILITY_CONTROL_HPP
#define VISIBILITY_CONTROL_HPP

// DLL Export/Import Macros for Cross-Platform Component Library

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MANIPULATION_AGX_ARM_MOVEIT_EXPORT __attribute__((dllexport))
    #define MANIPULATION_AGX_ARM_MOVEIT_IMPORT __attribute__((dllimport))
  #else
    #define MANIPULATION_AGX_ARM_MOVEIT_EXPORT __declspec(dllexport)
    #define MANIPULATION_AGX_ARM_MOVEIT_IMPORT __declspec(dllimport)
  #endif
  #ifdef MANIPULATION_AGX_ARM_MOVEIT_BUILDING_DLL
    #define MANIPULATION_AGX_ARM_MOVEIT_PUBLIC MANIPULATION_AGX_ARM_MOVEIT_EXPORT
  #else
    #define MANIPULATION_AGX_ARM_MOVEIT_PUBLIC MANIPULATION_AGX_ARM_MOVEIT_IMPORT
  #endif
  #define MANIPULATION_AGX_ARM_MOVEIT_LOCAL
#else
  #define MANIPULATION_AGX_ARM_MOVEIT_EXPORT __attribute__((visibility("default")))
  #define MANIPULATION_AGX_ARM_MOVEIT_IMPORT
  #if __GNUC__ >= 4
    #define MANIPULATION_AGX_ARM_MOVEIT_PUBLIC __attribute__((visibility("default")))
    #define MANIPULATION_AGX_ARM_MOVEIT_LOCAL __attribute__((visibility("hidden")))
  #else
    #define MANIPULATION_AGX_ARM_MOVEIT_PUBLIC
    #define MANIPULATION_AGX_ARM_MOVEIT_LOCAL
  #endif
#endif

#endif // VISIBILITY_CONTROL_HPP