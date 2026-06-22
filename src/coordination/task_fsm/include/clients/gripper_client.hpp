#ifndef TASK_FSM_NODE__CLIENTS__GRIPPER_CLIENT_HPP_
#define TASK_FSM_NODE__CLIENTS__GRIPPER_CLIENT_HPP_

#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <agx_motion_msgs/msg/gripper_cmd.hpp>
#include <agx_motion_msgs/msg/gripper_control_status.hpp>

#include "types/task_types.hpp"

namespace coordination
{

    /**
     * @brief 夹爪控制客户端。
     *
     * 负责：
     * - 发布夹爪控制命令；
     * - 订阅夹爪执行状态；
     * - 维护当前动作状态与活跃请求 ID；
     * - 支持超时检测与完成回调通知。
     */
    class GripperClient
    {
    public:
        /**
         * @brief 动作完成回调类型。
         * @param status 动作最终状态（成功/失败/超时等）。
         * @param message 结果说明文本。
         */
        using CompletionCallback = std::function<void(ActionStatus, const std::string &)>;

        /**
         * @brief 构造函数。
         * @param node 生命周期节点引用（不持有所有权）。
         * @param gripper_cmd_topic 夹爪命令发布话题名。
         * @param gripper_status_topic 夹爪状态订阅话题名。
         */
        GripperClient(
            rclcpp_lifecycle::LifecycleNode &node,
            const std::string &gripper_cmd_topic,
            const std::string &gripper_status_topic);

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
         * @brief 发送夹爪命令。
         * @param cmd 夹爪命令消息。
         * @return true 表示命令已受理；false 表示当前状态不允许或发送失败。
         */
        bool send(const agx_motion_msgs::msg::GripperCmd &cmd);

        /**
         * @brief 重置内部状态到初始空闲态。
         */
        void reset();

        /**
         * @brief 检查当前 pending 动作是否超时。
         * @param timeout 超时时长阈值。
         */
        void checkTimeout(std::chrono::milliseconds timeout);

    private:
        /**
         * @brief 夹爪状态回调。
         *
         * 解析状态消息并更新内部状态；必要时触发完成回调。
         */
        void onStatus(const agx_motion_msgs::msg::GripperControlStatus::SharedPtr msg);

        /// 生命周期节点引用（外部管理）。
        rclcpp_lifecycle::LifecycleNode &node_;
        /// 夹爪命令发布器。
        rclcpp::Publisher<agx_motion_msgs::msg::GripperCmd>::SharedPtr cmd_pub_;
        /// 夹爪状态订阅器。
        rclcpp::Subscription<agx_motion_msgs::msg::GripperControlStatus>::SharedPtr status_sub_;

        /// 并发访问保护锁。
        mutable std::mutex mutex_;
        /// 当前动作状态。
        ActionStatus status_{ActionStatus::IDLE};
        /// 当前活跃请求 ID（用于状态匹配）。
        std::string active_request_id_;
        /// 期望的闭合命令类型（默认 OPEN）。
        uint8_t expected_close_cmd_{agx_motion_msgs::msg::GripperCmd::CMD_OPEN};
        /// 是否等待抓取确认标志。
        bool wait_grasp_{false};
        /// 动作完成回调。
        CompletionCallback completion_cb_;
        /// 当前 pending 动作开始时间（用于超时检测）。
        std::optional<std::chrono::steady_clock::time_point> pending_since_;
    };

} // namespace coordination

#endif // TASK_FSM_NODE__CLIENTS__GRIPPER_CLIENT_HPP_