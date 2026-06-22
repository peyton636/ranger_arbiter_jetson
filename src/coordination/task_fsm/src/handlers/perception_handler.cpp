#include "handlers/perception_handler.hpp"

namespace coordination
{

    PerceptionHandler::PerceptionHandler(
        rclcpp_lifecycle::LifecycleNode &node,
        const std::string &object_pose_topic,
        const std::string &grasp_pose_topic,
        const std::string &output_frame)
        : node_(node), output_frame_(output_frame)
    {
        object_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseArray>(
            object_pose_topic, 10,
            std::bind(&PerceptionHandler::onObjectPoses, this, std::placeholders::_1));
        grasp_pose_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(grasp_pose_topic, 10);
    }

    bool PerceptionHandler::hasTargets() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_data_ && !latest_poses_.poses.empty();
    }

    size_t PerceptionHandler::targetCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_poses_.poses.size();
    }

    std::optional<geometry_msgs::msg::PoseStamped> PerceptionHandler::selectBestTarget()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_data_ || latest_poses_.poses.empty())
        {
            return std::nullopt;
        }

        geometry_msgs::msg::PoseStamped selected;
        selected.header = latest_poses_.header;
        if (selected.header.frame_id.empty())
        {
            selected.header.frame_id = output_frame_;
        }
        selected.pose = latest_poses_.poses.front();
        return selected;
    }

    void PerceptionHandler::publishSelectedGraspPose(const geometry_msgs::msg::PoseStamped &pose)
    {
        grasp_pose_pub_->publish(pose);
    }

    void PerceptionHandler::resetSelection()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_poses_.poses.clear();
        has_data_ = false;
    }

    void PerceptionHandler::onObjectPoses(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_poses_ = *msg;
        has_data_ = true;
    }

} // namespace coordination
