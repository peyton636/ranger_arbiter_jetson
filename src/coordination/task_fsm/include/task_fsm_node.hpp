/// @file task_fsm_node.hpp
/// @brief Pick&Place 任务编排的 ROS2 Lifecycle 节点入口。
///
/// 职责：
/// - 声明并加载参数；
/// - 组装 Policy、各子系统 Client 与 StateMachine；
/// - 提供 start/cancel/reset 服务；
/// - 通过定时器周期驱动 StateMachine::tick()。
///
/// 生命周期：
/// - on_configure: 创建资源与依赖；
/// - on_activate: 启动定时驱动；
/// - on_deactivate: 停止定时驱动；
/// - on_cleanup/on_shutdown: 释放资源。

#ifndef TASK_FSM_NODE__TASK_FSM_NODE_HPP_
#define TASK_FSM_NODE__TASK_FSM_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "clients/gripper_client.hpp"
#include "clients/motion_client.hpp"
#include "clients/navigation_client.hpp"
#include "fsm/state_machine.hpp"
#include "handlers/perception_handler.hpp"
#include "policy/pick_place_policy.hpp"
#include "visibility_control.hpp"

namespace coordination
{

    /**
     * @brief Task FSM 生命周期节点。
     *
     * 该节点作为任务编排入口，负责在 ROS2 生命周期内管理
     * 状态机及其依赖对象，并对外暴露简单控制服务。
     */
    class COORDINATION_TASK_FSM_PUBLIC TaskFsmNode
        : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        /**
         * @brief 构造函数。
         * @param options ROS2 节点启动选项。
         */
        explicit TaskFsmNode(const rclcpp::NodeOptions &options);

        /// @brief 析构函数。
        ~TaskFsmNode() override = default;

        /// @brief Lifecycle 回调返回类型别名。
        using CallbackReturn =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        /**
         * @brief 配置阶段回调：声明参数、加载配置并创建依赖对象。
         */
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief 激活阶段回调：启动定时器与对外服务能力。
         */
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief 去激活阶段回调：停止定时器，暂停任务推进。
         */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief 清理阶段回调：释放已创建资源，回到可重新配置状态。
         */
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief 关闭阶段回调：执行最终清理。
         */
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:
        /**
         * @brief 声明节点参数（含默认值）。
         */
        void declareParameters();

        /**
         * @brief 从参数服务器读取并组装策略配置。
         * @return PickPlacePolicyConfig 策略配置快照。
         */
        PickPlacePolicyConfig loadPolicyConfig();

        /**
         * @brief 定时器回调：驱动状态机执行一次 tick。
         */
        void onTick();

        /// 策略配置缓存（由参数加载得到）。
        PickPlacePolicyConfig policy_config_;

        /// Pick&Place 策略构建器。
        std::unique_ptr<PickPlacePolicy> policy_;

        /// 运动子系统客户端。
        std::shared_ptr<MotionClient> motion_client_;
        /// 夹爪子系统客户端。
        std::shared_ptr<GripperClient> gripper_client_;
        /// 导航子系统客户端。
        std::shared_ptr<NavigationClient> navigation_client_;
        /// 感知处理器。
        std::shared_ptr<PerceptionHandler> perception_handler_;

        /// 主任务状态机。
        std::unique_ptr<StateMachine> state_machine_;

        /// 周期 tick 定时器。
        rclcpp::TimerBase::SharedPtr tick_timer_;

        /// 启动任务服务（Trigger）。
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
        /// 取消任务服务（Trigger）。
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
        /// 复位任务服务（Trigger）。
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    };

} // namespace coordination

#endif // TASK_FSM_NODE__TASK_FSM_NODE_HPP_