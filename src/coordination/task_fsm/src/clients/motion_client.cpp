#include "clients/motion_client.hpp"

namespace coordination
{

    MotionClient::MotionClient(
        rclcpp_lifecycle::LifecycleNode &node,
        const std::string &plan_request_topic,
        const std::string &execute_feedback_topic)
        : node_(node)
    {
        plan_pub_ = node_.create_publisher<agx_motion_msgs::msg::PlanRequest>(plan_request_topic, 10);
        feedback_sub_ = node_.create_subscription<agx_motion_msgs::msg::ExecuteFeedback>(
            execute_feedback_topic, 10,
            std::bind(&MotionClient::onFeedback, this, std::placeholders::_1));
    }

    void MotionClient::setCompletionCallback(CompletionCallback cb)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        completion_cb_ = std::move(cb);
    }

    ActionStatus MotionClient::status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

    std::string MotionClient::activeRequestId() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_request_id_;
    }

    bool MotionClient::send(const agx_motion_msgs::msg::PlanRequest &request)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_ == ActionStatus::PENDING)
        {
            RCLCPP_WARN(node_.get_logger(), "MotionClient busy, reject request_id=%s", request.request_id.c_str());
            return false;
        }
        active_request_id_ = request.request_id;
        status_ = ActionStatus::PENDING;
        pending_since_ = std::chrono::steady_clock::now();
        plan_pub_->publish(request);
        return true;
    }

    void MotionClient::reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = ActionStatus::IDLE;
        active_request_id_.clear();
        pending_since_.reset();
    }

    void MotionClient::checkTimeout(std::chrono::milliseconds timeout)
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
            cb(ActionStatus::TIMEOUT, "motion timeout");
        }
    }

    void MotionClient::onFeedback(const agx_motion_msgs::msg::ExecuteFeedback::SharedPtr msg)
    {
        CompletionCallback cb;
        ActionStatus final_status = ActionStatus::IDLE;
        std::string message;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (status_ != ActionStatus::PENDING || msg->request_id != active_request_id_)
            {
                return;
            }

            if (msg->status == agx_motion_msgs::msg::ExecuteFeedback::STATUS_SUCCEEDED)
            {
                status_ = ActionStatus::SUCCEEDED;
                final_status = ActionStatus::SUCCEEDED;
                message = msg->message;
                pending_since_.reset();
                cb = completion_cb_;
            }
            else if (
                msg->status == agx_motion_msgs::msg::ExecuteFeedback::STATUS_FAILED ||
                msg->status == agx_motion_msgs::msg::ExecuteFeedback::STATUS_ABORTED)
            {
                status_ = ActionStatus::FAILED;
                final_status = ActionStatus::FAILED;
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
