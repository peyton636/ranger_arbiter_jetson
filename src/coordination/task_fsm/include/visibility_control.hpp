#ifndef TASK_FSM_NODE__VISIBILITY_CONTROL_HPP_
#define TASK_FSM_NODE__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COORDINATION_TASK_FSM_EXPORT __attribute__((dllexport))
#define COORDINATION_TASK_FSM_IMPORT __attribute__((dllimport))
#else
#define COORDINATION_TASK_FSM_EXPORT __declspec(dllexport)
#define COORDINATION_TASK_FSM_IMPORT __declspec(dllimport)
#endif
#ifdef COORDINATION_TASK_FSM_BUILDING_DLL
#define COORDINATION_TASK_FSM_PUBLIC TASK_FSM_EXPORT
#else
#define COORDINATION_TASK_FSM_PUBLIC TASK_FSM_IMPORT
#endif
#else
#define COORDINATION_TASK_FSM_EXPORT __attribute__((visibility("default")))
#define COORDINATION_TASK_FSM_IMPORT
#if __GNUC__ >= 4
#define COORDINATION_TASK_FSM_PUBLIC __attribute__((visibility("default")))
#else
#define TASK_FSM_PUBLIC
#endif
#endif

#endif // TASK_FSM_NODE__VISIBILITY_CONTROL_HPP_
