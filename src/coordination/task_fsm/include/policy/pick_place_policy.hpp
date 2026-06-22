#ifndef TASK_FSM_NODE__POLICY__PICK_PLACE_POLICY_HPP_
#define TASK_FSM_NODE__POLICY__PICK_PLACE_POLICY_HPP_

#include <chrono>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <agx_motion_msgs/msg/gripper_cmd.hpp>
#include <agx_motion_msgs/msg/plan_request.hpp>

#include "types/task_types.hpp"

namespace coordination
{

    /**
     * @brief Pick&Place 策略配置。
     *
     * 该结构体集中定义流程中用到的命名位、速度参数、夹爪参数、
     * 超时参数和重试策略，便于统一管理与参数化调优。
     */
    struct PickPlacePolicyConfig
    {
        /// MoveIt 规划组名称（机械臂）。
        std::string arm_group{"arm"};
        /// 默认坐标系（用于姿态/方向目标构建）。
        std::string frame_id{"base_link"};

        /// 回零位（命名目标）。
        std::string named_home{"home"};
        /// 抓取准备位（命名目标）。
        std::string named_grasp_ready{"grasp_ready"};
        /// 预抓取位（命名目标）。
        std::string named_pre_grasp{"pre_grasp"};
        /// 撤退位（命名目标）。
        std::string named_retreat{"retreat"};

        /// 速度缩放（0~1）。
        double velocity_scaling{0.15};
        /// 加速度缩放（0~1）。
        double acceleration_scaling{0.15};

        /// 抓取逼近距离（米）。
        double approach_dist_m{0.02};
        /// 笛卡尔路径步长（米）。
        double approach_step_m{0.005};
        /// 放置后撤退距离（米）。
        double place_retreat_dist_m{0.03};

        /// 开爪宽度（米）。
        double gripper_open_width{0.08};
        /// 闭爪力（单位由下游驱动定义）。
        double gripper_close_force{20.0};

        /// 通用重试策略（次数/间隔等）。
        RetryPolicy retry{};

        /// 感知等待超时。
        std::chrono::milliseconds perception_timeout{8000};
        /// 机械臂规划执行超时。
        std::chrono::milliseconds motion_timeout{60000};
        /// 夹爪动作超时。
        std::chrono::milliseconds gripper_timeout{10000};
        /// 导航超时。
        std::chrono::milliseconds navigation_timeout{120000};

        /// 是否跳过真实导航（调试/仿真模式常用）。
        bool skip_navigation{true};
    };

    /**
     * @brief Pick&Place 策略构建器。
     *
     * 根据配置生成统一格式的动作请求消息（运动规划、夹爪命令、导航目标等）。
     */
    class PickPlacePolicy
    {
    public:
        /**
         * @brief 构造策略对象。
         * @param config 策略配置（值语义保存）。
         */
        explicit PickPlacePolicy(PickPlacePolicyConfig config);

        /**
         * @brief 获取只读配置引用。
         */
        [[nodiscard]] const PickPlacePolicyConfig &config() const { return config_; }

        /**
         * @brief 生成命名目标规划请求。
         * @param request_id 请求 ID（用于链路追踪）。
         * @param named_target 命名目标名（如 home / pre_grasp）。
         * @param execute 是否直接执行。
         */
        [[nodiscard]] agx_motion_msgs::msg::PlanRequest makeNamedPlan(
            const std::string &request_id,
            const std::string &named_target,
            bool execute = true) const;

        /**
         * @brief 生成位姿目标规划请求。
         * @param request_id 请求 ID。
         * @param pose 目标位姿。
         * @param execute 是否直接执行。
         */
        [[nodiscard]] agx_motion_msgs::msg::PlanRequest makePosePlan(
            const std::string &request_id,
            const geometry_msgs::msg::PoseStamped &pose,
            bool execute = true) const;

        /**
         * @brief 生成笛卡尔方向位移规划请求。
         * @param request_id 请求 ID。
         * @param direction 位移方向（Vector3Stamped，含 frame_id）。
         * @param distance_m 位移距离（米）。
         * @param execute 是否直接执行。
         */
        [[nodiscard]] agx_motion_msgs::msg::PlanRequest makeCartesianPlan(
            const std::string &request_id,
            const geometry_msgs::msg::Vector3Stamped &direction,
            double distance_m,
            bool execute = true) const;

        /**
         * @brief 生成夹爪命令。
         * @param request_id 请求 ID。
         * @param command 命令字（开/关等，取值见 GripperCmd 定义）。
         * @param wait_grasp 是否等待抓取确认。
         */
        [[nodiscard]] agx_motion_msgs::msg::GripperCmd makeGripperCmd(
            const std::string &request_id,
            uint8_t command,
            bool wait_grasp = false) const;

        /**
         * @brief 生成 TCP 向下方向向量（常用于抓取逼近）。
         */
        [[nodiscard]] geometry_msgs::msg::Vector3Stamped makeTcpDownDirection() const;

        /**
         * @brief 生成 TCP 向上方向向量（常用于撤退）。
         */
        [[nodiscard]] geometry_msgs::msg::Vector3Stamped makeTcpUpDirection() const;

        /**
         * @brief 生成平面导航目标（x, y, yaw）。
         * @param x 目标 x（米）。
         * @param y 目标 y（米）。
         * @param yaw 目标偏航角（弧度）。
         */
        [[nodiscard]] geometry_msgs::msg::PoseStamped makeNavigationGoal(
            double x, double y, double yaw) const;

    private:
        /// 策略配置快照。
        PickPlacePolicyConfig config_;
    };

} // namespace coordination

#endif // TASK_FSM_NODE__POLICY__PICK_PLACE_POLICY_HPP_