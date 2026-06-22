#ifndef TARGET_TRACKER__VISIBILITY_CONTROL_HPP_
#define TARGET_TRACKER__VISIBILITY_CONTROL_HPP_

// 组件库 DLL 导出/导入宏（跨平台）

#if defined _WIN32 || defined __CYGWIN__
  #ifdef TARGET_TRACKER_BUILDING_DLL
    #define TARGET_TRACKER_PUBLIC __declspec(dllexport)
  #else
    #define TARGET_TRACKER_PUBLIC __declspec(dllimport)
  #endif
#else
  #define TARGET_TRACKER_PUBLIC __attribute__((visibility("default")))
#endif

#endif  // TARGET_TRACKER__VISIBILITY_CONTROL_HPP_
