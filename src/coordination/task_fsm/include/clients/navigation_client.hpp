#ifndef TASK_FSM_NODE__CLIENTS__NAVIGATION_CLIENT_HPP_
#define TASK_FSM_NODE__CLIENTS__NAVIGATION_CLIENT_HPP_

#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "types/task_types.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace coordination
{

    /**
     * @brief 导航客户端：负责发送导航目标、接收到达反馈、维护动作状态与超时检测。
     */
    class NavigationClient
    {
    public:
        /**
         * @brief 动作完成回调。
         * @param status 动作最终状态。
         * @param message 结果说明（成功信息或失败原因）。
         */
        using CompletionCallback = std::function<void(ActionStatus, const std::string &)>;

        /**
         * @brief 构造函数。
         * @param node 生命周期节点引用（由外部管理）。
         * @param nav_goal_topic 导航目标发布话题。
         * @param nav_arrived_topic 导航到达订阅话题（Bool）。
         * @param skip_navigation 是否跳过真实导航（用于调试/仿真场景）。
         */
        NavigationClient(
            rclcpp_lifecycle::LifecycleNode &node,
            const std::string &nav_goal_topic,
            const std::string &nav_arrived_topic,
            bool skip_navigation);

        /**
         * @brief 设置动作完成回调（覆盖旧回调）。
         */
        void setCompletionCallback(CompletionCallback cb);

        /**
         * @brief 获取当前导航动作状态（线程安全）。
         */
        ActionStatus status() const;

        /**
         * @brief 发送导航目标。
         * @param goal 目标位姿（PoseStamped）。
         * @return true 表示目标已受理；false 表示发送失败或当前状态不允许。
         */
        bool sendGoal(const geometry_msgs::msg::PoseStamped &goal);

        /**
         * @brief 重置内部状态到空闲态。
         */
        void reset();

        /**
         * @brief 检查 pending 导航请求是否超时。
         * @param timeout 超时时长阈值。
         */
        void checkTimeout(std::chrono::milliseconds timeout);

    private:
        /**
         * @brief 到达反馈回调：处理导航完成信号并更新内部状态。
         * @param msg 到达标志（true=到达，false=未到达/失败语义由上层约定）。
         */
        void onArrived(const std_msgs::msg::Bool::SharedPtr msg);

        /// ROS2 生命周期节点引用（不拥有）。
        rclcpp_lifecycle::LifecycleNode &node_;
        /// 是否跳过导航执行。
        bool skip_navigation_;
        /// 导航目标发布器。
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        /// 导航到达反馈订阅器。
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;

        /// 并发状态保护锁。
        mutable std::mutex mutex_;
        /// 当前导航动作状态。
        ActionStatus status_{ActionStatus::IDLE};
        /// 动作完成回调。
        CompletionCallback completion_cb_;
        /// 当前 pending 导航动作开始时间，用于超时判断。
        std::optional<std::chrono::steady_clock::time_point> pending_since_;
    };

} // namespace coordination

#endif // TASK_FSM_NODE__CLIENTS__NAVIGATION_CLIENT_HPP_