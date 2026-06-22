#ifndef TASK_FSM_NODE__FSM__STATE_MACHINE_HPP_
#define TASK_FSM_NODE__FSM__STATE_MACHINE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "clients/gripper_client.hpp"
#include "clients/motion_client.hpp"
#include "clients/navigation_client.hpp"
#include "handlers/perception_handler.hpp"
#include "policy/pick_place_policy.hpp"
#include "types/task_context.hpp"

namespace coordination
{

    /**
     * @brief 任务有限状态机（FSM）。
     *
     * 负责调度抓取放置流程：
     * - 导航到抓取位；
     * - 感知与目标选择；
     * - 机械臂运动与夹爪控制；
     * - 导航到放置位并完成放置；
     * - 异常重试、失败收敛与完成收敛。
     */
    class StateMachine
    {
    public:
        /**
         * @brief 构造状态机并注入依赖模块。
         * @param node 生命周期节点引用（不持有所有权）。
         * @param policy 抓取放置策略配置（值语义存储）。
         * @param motion 运动客户端。
         * @param gripper 夹爪客户端。
         * @param navigation 导航客户端。
         * @param perception 感知处理器。
         */
        StateMachine(
            rclcpp_lifecycle::LifecycleNode &node,
            PickPlacePolicy policy,
            std::shared_ptr<MotionClient> motion,
            std::shared_ptr<GripperClient> gripper,
            std::shared_ptr<NavigationClient> navigation,
            std::shared_ptr<PerceptionHandler> perception);

        /**
         * @brief 状态机周期执行入口。
         *
         * 由外部定时调用，用于推进当前状态对应的 step 逻辑。
         */
        void tick();

        /**
         * @brief 请求启动一次 Pick&Place 流程。
         *
         * 通常在 IDLE 态触发，内部会设置上下文并进入首个执行态。
         */
        void requestStartPickPlace();

        /**
         * @brief 请求取消当前任务。
         *
         * 一般会设置取消标志，并在合适时机转入恢复或空闲状态。
         */
        void requestCancel();

        /**
         * @brief 请求复位状态机上下文。
         *
         * 用于错误后清理现场并回到可重新启动的状态。
         */
        void requestReset();

        /**
         * @brief 获取当前 FSM 状态。
         */
        TaskFsmState state() const;

        /**
         * @brief 获取任务上下文只读引用。
         */
        const TaskContext &context() const;

    private:
        /// 状态处理函数指针类型。
        using StepFn = void (StateMachine::*)();

        /**
         * @brief 绑定各子系统完成回调到统一收敛入口。
         */
        void bindClientCallbacks();

        /**
         * @brief 发布当前状态（用于外部监控/可视化）。
         */
        void publishState();

        /**
         * @brief 状态迁移通用入口。
         * @param next 目标状态。
         * @param reason 迁移原因（可选，便于日志与诊断）。
         */
        void transitionTo(TaskFsmState next, const std::string &reason = "");

        /**
         * @brief 进入失败路径并记录原因。
         * @param reason 失败说明。
         */
        void failWith(const std::string &reason);

        /**
         * @brief 发起运动子任务并设置等待状态。
         * @return true 发起成功；false 发起失败。
         */
        bool beginMotion(const agx_motion_msgs::msg::PlanRequest &req);

        /**
         * @brief 发起夹爪子任务并设置等待状态。
         * @return true 发起成功；false 发起失败。
         */
        bool beginGripper(const agx_motion_msgs::msg::GripperCmd &cmd);

        /**
         * @brief 发起导航子任务并设置等待状态。
         * @return true 发起成功；false 发起失败。
         */
        bool beginNavigation(const geometry_msgs::msg::PoseStamped &goal);

        /**
         * @brief 子系统完成统一回调。
         *
         * motion/gripper/navigation 完成后均汇聚到此函数，
         * 由当前状态决定下一步迁移逻辑。
         */
        void onSubSystemDone(ActionStatus status, const std::string &message);

        /// IDLE 状态处理。
        void stepIdle();
        /// 导航到抓取点状态处理。
        void stepNavigateToPick();
        /// 等待感知结果状态处理。
        void stepWaitPerception();
        /// 从候选中选择目标状态处理。
        void stepSelectTarget();
        /// 打开夹爪状态处理。
        void stepOpenGripper();
        /// 运动到抓取准备位状态处理。
        void stepMoveGraspReady();
        /// 运动到预抓位状态处理。
        void stepMovePreGrasp();
        /// 笛卡尔逼近状态处理。
        void stepApproachCartesian();
        /// 闭合夹爪状态处理。
        void stepCloseGripper();
        /// 抓取后撤退状态处理。
        void stepRetreat();
        /// 导航到放置点状态处理。
        void stepNavigateToPlace();
        /// 放置阶段打开夹爪状态处理。
        void stepPlaceOpenGripper();
        /// 放置后撤退状态处理。
        void stepPlaceRetreat();
        /// 回 Home 位状态处理。
        void stepGoHome();
        /// 重试恢复状态处理。
        void stepRetryRecovery();
        /// 错误状态处理。
        void stepError();
        /// 完成状态处理。
        void stepCompleted();

        /// 生命周期节点引用（外部管理）。
        rclcpp_lifecycle::LifecycleNode &node_;
        /// 抓取放置策略参数。
        PickPlacePolicy policy_;
        /// 任务运行上下文（状态、标志、中间结果等）。
        TaskContext ctx_;

        /// 运动客户端。
        std::shared_ptr<MotionClient> motion_;
        /// 夹爪客户端。
        std::shared_ptr<GripperClient> gripper_;
        /// 导航客户端。
        std::shared_ptr<NavigationClient> navigation_;
        /// 感知处理器。
        std::shared_ptr<PerceptionHandler> perception_;

        /// FSM 状态发布器。
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
        /// 抓取点导航目标。
        geometry_msgs::msg::PoseStamped pick_nav_goal_;
        /// 放置点导航目标。
        geometry_msgs::msg::PoseStamped place_nav_goal_;
        /// 请求计数器（用于生成任务/请求序号）。
        uint64_t request_counter_{0};
    };

} // namespace coordination

#endif // TASK_FSM_NODE__FSM__STATE_MACHINE_HPP_