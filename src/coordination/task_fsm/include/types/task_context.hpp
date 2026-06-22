#ifndef TASK_FSM_NODE__TYPES__TASK_CONTEXT_HPP_
#define TASK_FSM_NODE__TYPES__TASK_CONTEXT_HPP_

#include <chrono>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "types/task_types.hpp"

namespace coordination
{

    /**
     * @brief 任务运行上下文。
     *
     * 用于在 FSM 各状态之间共享运行时数据，包括：
     * - 当前状态与待处理控制命令；
     * - 各子系统（运动/夹爪/导航）执行状态；
     * - 当前请求 ID 与最近错误信息；
     * - 各阶段重试计数；
     * - 感知选中目标与状态超时信息。
     */
    struct TaskContext
    {
        /// 当前 FSM 状态。
        TaskFsmState state{TaskFsmState::IDLE};
        /// 待处理任务命令（启动/取消/复位等）。
        TaskCommand pending_command{TaskCommand::NONE};

        /// 运动子系统状态。
        ActionStatus motion_status{ActionStatus::IDLE};
        /// 夹爪子系统状态。
        ActionStatus gripper_status{ActionStatus::IDLE};
        /// 导航子系统状态。
        ActionStatus navigation_status{ActionStatus::IDLE};

        /// 当前活跃请求 ID（用于链路追踪与日志关联）。
        std::string active_request_id;
        /// 最近一次失败原因描述。
        std::string last_error;

        /// 感知阶段重试次数。
        int perception_retry_count{0};
        /// 运动阶段重试次数。
        int motion_retry_count{0};
        /// 夹爪阶段重试次数。
        int gripper_retry_count{0};
        /// 导航阶段重试次数。
        int navigation_retry_count{0};

        /// 当前选中的抓取位姿（无目标时为空）。
        std::optional<geometry_msgs::msg::PoseStamped> selected_grasp_pose;
        /// 当前状态进入时间点。
        std::optional<std::chrono::steady_clock::time_point> state_enter_time;
        /// 当前等待截止时间点（用于超时判断）。
        std::optional<std::chrono::steady_clock::time_point> wait_deadline;

        /**
         * @brief 为新任务重置上下文。
         *
         * 通常会清理请求 ID、错误信息、重试计数和中间结果，
         * 并将子系统状态恢复到初始值。
         */
        void resetForNewTask();

        /**
         * @brief 进入新状态并更新时间戳。
         * @param next 目标状态。
         */
        void enterState(TaskFsmState next);

        /**
         * @brief 以“当前时刻 + timeout”设置等待截止时间。
         * @param timeout 超时时长。
         */
        void setDeadlineFromNow(std::chrono::milliseconds timeout);

        /**
         * @brief 判断当前等待截止时间是否已过期。
         * @return true 已过期；false 未过期或未设置截止时间。
         */
        bool isDeadlineExpired() const;
    };

} // namespace coordination

#endif // TASK_FSM_NODE__TYPES__TASK_CONTEXT_HPP_