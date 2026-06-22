#ifndef TASK_FSM_NODE__CLIENTS__MOTION_CLIENT_HPP_
#define TASK_FSM_NODE__CLIENTS__MOTION_CLIENT_HPP_

#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include <agx_motion_msgs/msg/execute_feedback.hpp>
#include <agx_motion_msgs/msg/plan_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "types/task_types.hpp"

namespace coordination
{

    /**
     * @brief 运动规划/执行客户端：负责发送规划请求、接收执行反馈、维护动作状态与超时检测。
     */
    class MotionClient
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
         * @param node 生命周期节点引用（由外部管理生命周期）。
         * @param plan_request_topic 规划请求发布话题。
         * @param execute_feedback_topic 执行反馈订阅话题。
         */
        MotionClient(
            rclcpp_lifecycle::LifecycleNode &node,
            const std::string &plan_request_topic,
            const std::string &execute_feedback_topic);

        /**
         * @brief 设置动作完成回调（覆盖旧回调）。
         */
        void setCompletionCallback(CompletionCallback cb);

        /**
         * @brief 获取当前动作状态（线程安全）。
         */
        ActionStatus status() const;

        /**
         * @brief 获取当前活跃请求 ID（线程安全）。
         * @return 空字符串表示当前无活跃请求。
         */
        std::string activeRequestId() const;

        /**
         * @brief 发送一次规划/执行请求。
         * @param request 规划请求消息。
         * @return true 表示请求已成功发送并进入执行；false 表示发送失败或状态不允许发送。
         */
        bool send(const agx_motion_msgs::msg::PlanRequest &request);

        /**
         * @brief 重置内部状态到初始空闲态。
         */
        void reset();

        /**
         * @brief 检查当前 pending 请求是否超时。
         * @param timeout 超时时长阈值。
         */
        void checkTimeout(std::chrono::milliseconds timeout);

    private:
        /**
         * @brief 执行反馈回调：处理执行进度/结果并更新内部状态。
         * @param msg 执行反馈消息。
         */
        void onFeedback(const agx_motion_msgs::msg::ExecuteFeedback::SharedPtr msg);

        /// ROS2 生命周期节点引用（不拥有）。
        rclcpp_lifecycle::LifecycleNode &node_;
        /// 规划请求发布器。
        rclcpp::Publisher<agx_motion_msgs::msg::PlanRequest>::SharedPtr plan_pub_;
        /// 执行反馈订阅器。
        rclcpp::Subscription<agx_motion_msgs::msg::ExecuteFeedback>::SharedPtr feedback_sub_;

        /// 并发状态保护锁。
        mutable std::mutex mutex_;
        /// 当前动作状态。
        ActionStatus status_{ActionStatus::IDLE};
        /// 当前活跃请求 ID（用于反馈匹配与状态跟踪）。
        std::string active_request_id_;
        /// 动作完成回调。
        CompletionCallback completion_cb_;
        /// 当前 pending 动作开始时间，用于超时判断。
        std::optional<std::chrono::steady_clock::time_point> pending_since_;
    };

} // namespace coordination

#endif // TASK_FSM_NODE__CLIENTS__MOTION_CLIENT_HPP_