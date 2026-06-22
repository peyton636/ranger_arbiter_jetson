#include "clients/gripper_client.hpp"

namespace coordination
{

    GripperClient::GripperClient(rclcpp_lifecycle::LifecycleNode &node,
                                 const std::string &gripper_cmd_topic,
                                 const std::string &gripper_status_topic)
        : node_(node)
    {
        cmd_pub_ = node_.create_publisher<agx_motion_msgs::msg::GripperCmd>(
            gripper_cmd_topic, 10);
        status_sub_ = node_.create_subscription<agx_motion_msgs::msg::GripperControlStatus>(
            gripper_status_topic, 10,
            std::bind(&GripperClient::onStatus, this, std::placeholders::_1));
    }

    void GripperClient::setCompletionCallback(CompletionCallback cb)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        completion_cb_ = std::move(cb);
    }

    ActionStatus GripperClient::status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

    std::string GripperClient::activeRequestId() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_request_id_;
    }

    bool GripperClient::send(const agx_motion_msgs::msg::GripperCmd &cmd)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_ == ActionStatus::PENDING)
        {
            RCLCPP_WARN(node_.get_logger(), "GripperClient busy, reject request_id=%s",
                        cmd.request_id.c_str());
            return false;
        }
        active_request_id_ = cmd.request_id;
        expected_close_cmd_ = cmd.command;
        wait_grasp_ = cmd.wait_grasp;
        status_ = ActionStatus::PENDING;
        pending_since_ = std::chrono::steady_clock::now();
        cmd_pub_->publish(cmd);
        return true;
    }

    void GripperClient::reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = ActionStatus::IDLE;
        active_request_id_.clear();
        pending_since_.reset();
    }

    void GripperClient::checkTimeout(std::chrono::milliseconds timeout)
    {
        CompletionCallback cb;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (status_ != ActionStatus::PENDING || !pending_since_.has_value())
            {
                return;
            }
            if (std::chrono::steady_clock::now() - pending_since_.value() < timeout)
            {
                return;
            }
            status_ = ActionStatus::TIMEOUT;
            pending_since_.reset();
            cb = completion_cb_;
        }
        if (cb)
        {
            cb(ActionStatus::TIMEOUT, "gripper timeout");
        }
    }

    void GripperClient::onStatus(
        const agx_motion_msgs::msg::GripperControlStatus::SharedPtr msg)
    {
        CompletionCallback cb;
        ActionStatus final_status = ActionStatus::IDLE;
        std::string message;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (status_ != ActionStatus::PENDING ||
                msg->request_id != active_request_id_)
            {
                return;
            }

            const bool close_done =
                expected_close_cmd_ == agx_motion_msgs::msg::GripperCmd::CMD_CLOSE &&
                msg->state == agx_motion_msgs::msg::GripperControlStatus::STATE_GRASPED;
            const bool open_done =
                expected_close_cmd_ == agx_motion_msgs::msg::GripperCmd::CMD_OPEN &&
                (msg->state == agx_motion_msgs::msg::GripperControlStatus::STATE_OPEN ||
                 msg->state == agx_motion_msgs::msg::GripperControlStatus::STATE_IDLE);
            const bool failed =
                msg->state == agx_motion_msgs::msg::GripperControlStatus::STATE_FAILED;

            if (failed)
            {
                status_ = ActionStatus::FAILED;
                final_status = ActionStatus::FAILED;
                message = msg->message;
                pending_since_.reset();
                cb = completion_cb_;
            }
            else if (close_done || open_done)
            {
                status_ = ActionStatus::SUCCEEDED;
                final_status = ActionStatus::SUCCEEDED;
                message = msg->message;
                pending_since_.reset();
                cb = completion_cb_;
            }
            else if (!wait_grasp_ &&
                     msg->state ==
                         agx_motion_msgs::msg::GripperControlStatus::STATE_IDLE)
            {
                status_ = ActionStatus::SUCCEEDED;
                final_status = ActionStatus::SUCCEEDED;
                message = msg->message;
                pending_since_.reset();
                cb = completion_cb_;
            }
        }

        if (cb)
        {
            cb(final_status, message);
        }
    }

} // namespace coordination
