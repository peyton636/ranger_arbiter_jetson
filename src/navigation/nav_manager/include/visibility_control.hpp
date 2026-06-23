#ifndef NAV_MANAGER__VISIBILITY_CONTROL_HPP_
#define NAV_MANAGER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef NAV_MANAGER_BUILDING_DLL
    #define NAV_MANAGER_PUBLIC __declspec(dllexport)
  #else
    #define NAV_MANAGER_PUBLIC __declspec(dllimport)
  #endif
#else
  #define NAV_MANAGER_PUBLIC __attribute__((visibility("default")))
#endif

#endif  // NAV_MANAGER__VISIBILITY_CONTROL_HPP_
