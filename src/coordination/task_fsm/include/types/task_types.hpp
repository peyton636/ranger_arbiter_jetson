#ifndef TASK_FSM_NODE__TYPES__TASK_TYPES_HPP_
#define TASK_FSM_NODE__TYPES__TASK_TYPES_HPP_

#include <cstdint>
#include <string>

namespace coordination
{

    /**
     * @brief FSM 主状态：描述 Pick&Place 流程中的离散阶段。
     */
    enum class TaskFsmState : uint8_t
    {
        IDLE = 0,           ///< 空闲，等待 START 命令。
        NAVIGATE_TO_PICK,   ///< 导航至抓取工位。
        WAIT_PERCEPTION,    ///< 等待感知发布目标。
        SELECT_TARGET,      ///< 从候选目标中选取抓取位姿。
        OPEN_GRIPPER,       ///< 张开夹爪。
        MOVE_GRASP_READY,   ///< 运动至 grasp_ready 命名位姿。
        MOVE_PRE_GRASP,     ///< 运动至预抓取位姿。
        APPROACH_CARTESIAN, ///< 笛卡尔直线接近物体。
        CLOSE_GRIPPER,      ///< 闭合夹爪并等待抓取确认。
        RETREAT,            ///< 抓取后撤离。
        NAVIGATE_TO_PLACE,  ///< 导航至放置工位。
        PLACE_OPEN_GRIPPER, ///< 放置时张开夹爪。
        PLACE_RETREAT,      ///< 放置后上抬撤离。
        GO_HOME,            ///< 回 home 位。
        RETRY_RECOVERY,     ///< 失败后恢复到安全点并重试。
        ERROR,              ///< 不可恢复错误，需 RESET。
        COMPLETED,          ///< 单次任务完成（下一次 tick 通常回到 IDLE）。
    };

    /**
     * @brief 子系统异步动作状态（运动/夹爪/导航共用）。
     */
    enum class ActionStatus : uint8_t
    {
        IDLE = 0,  ///< 空闲/未开始。
        PENDING,   ///< 执行中。
        SUCCEEDED, ///< 执行成功。
        FAILED,    ///< 执行失败。
        TIMEOUT,   ///< 执行超时。
    };

    /**
     * @brief 外部异步命令（由状态机在 tick 中消费）。
     */
    enum class TaskCommand : uint8_t
    {
        NONE = 0,         ///< 无命令。
        START_PICK_PLACE, ///< 启动一次 Pick&Place 流程。
        CANCEL,           ///< 取消当前任务。
        RESET,            ///< 复位状态机上下文。
    };

    /**
     * @brief 各子系统最大重试次数配置。
     */
    struct RetryPolicy
    {
        int max_perception_retries{3}; ///< 感知阶段最大重试次数。
        int max_motion_retries{2};     ///< 运动阶段最大重试次数。
        int max_gripper_retries{2};    ///< 夹爪阶段最大重试次数。
        int max_navigation_retries{2}; ///< 导航阶段最大重试次数。
    };

    /**
     * @brief FSM 状态转字符串（用于日志、监控与状态发布）。
     * @param state FSM 状态枚举值。
     * @return 对应的可读字符串。
     */
    std::string taskStateToString(TaskFsmState state);

    /**
     * @brief 动作状态转字符串（用于日志与诊断）。
     * @param status 动作状态枚举值。
     * @return 对应的可读字符串。
     */
    std::string actionStatusToString(ActionStatus status);

} // namespace coordination

#endif // TASK_FSM_NODE__TYPES__TASK_TYPES_HPP_