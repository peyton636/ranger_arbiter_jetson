#ifndef TASK_FSM_NODE__HANDLERS__PERCEPTION_HANDLER_HPP_
#define TASK_FSM_NODE__HANDLERS__PERCEPTION_HANDLER_HPP_

#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace coordination
{

    /**
     * @brief 感知结果处理器。
     *
     * 负责：
     * - 订阅目标物体位姿集合（PoseArray）；
     * - 缓存最新感知结果并提供查询接口；
     * - 从候选目标中选择一个“最佳抓取目标”；
     * - 发布选中的抓取位姿（PoseStamped）供下游模块使用。
     */
    class PerceptionHandler
    {
    public:
        /**
         * @brief 构造函数。
         * @param node 生命周期节点引用（不持有所有权）。
         * @param object_pose_topic 物体位姿订阅话题。
         * @param grasp_pose_topic 选中抓取位姿发布话题。
         * @param output_frame 输出位姿坐标系（frame_id）。
         */
        PerceptionHandler(rclcpp_lifecycle::LifecycleNode &node,
                          const std::string &object_pose_topic,
                          const std::string &grasp_pose_topic,
                          const std::string &output_frame);

        /**
         * @brief 当前是否有可用目标。
         * @return true 表示已收到且存在候选目标；false 表示暂无可用目标。
         */
        bool hasTargets() const;

        /**
         * @brief 当前候选目标数量。
         * @return 候选目标个数；无数据时通常返回 0。
         */
        size_t targetCount() const;

        /**
         * @brief 选择最佳目标并转换为抓取位姿。
         * @return 若选择成功返回 PoseStamped；否则返回 std::nullopt。
         */
        std::optional<geometry_msgs::msg::PoseStamped> selectBestTarget();

        /**
         * @brief 发布选中的抓取位姿。
         * @param pose 要发布的抓取位姿。
         */
        void publishSelectedGraspPose(const geometry_msgs::msg::PoseStamped &pose);

        /**
         * @brief 清空/重置当前选择相关状态。
         *
         * 不一定清除原始感知缓存，具体以实现为准。
         */
        void resetSelection();

    private:
        /**
         * @brief 物体位姿回调：接收并缓存最新感知结果。
         * @param msg 感知输出的位姿数组。
         */
        void onObjectPoses(const geometry_msgs::msg::PoseArray::SharedPtr msg);

        /// 生命周期节点引用（外部管理生命周期）。
        rclcpp_lifecycle::LifecycleNode &node_;
        /// 输出抓取位姿使用的坐标系。
        std::string output_frame_;

        /// 物体位姿订阅器。
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
            object_pose_sub_;
        /// 选中抓取位姿发布器。
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_pub_;

        /// 并发访问保护锁。
        mutable std::mutex mutex_;
        /// 最近一次接收到的感知位姿集合。
        geometry_msgs::msg::PoseArray latest_poses_;
        /// 是否已接收到至少一次有效感知数据。
        bool has_data_{false};
    };

} // namespace coordination

#endif // TASK_FSM_NODE__HANDLERS__PERCEPTION_HANDLER_HPP_