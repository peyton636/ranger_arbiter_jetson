#include "clients/navigation_client.hpp"

namespace coordination
{

    NavigationClient::NavigationClient(
        rclcpp_lifecycle::LifecycleNode &node,
        const std::string &nav_goal_topic,
        const std::string &nav_arrived_topic,
        bool skip_navigation)
        : node_(node), skip_navigation_(skip_navigation)
    {
        goal_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(nav_goal_topic, 10);
        arrived_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
            nav_arrived_topic, 10,
            std::bind(&NavigationClient::onArrived, this, std::placeholders::_1));
    }

    void NavigationClient::setCompletionCallback(CompletionCallback cb)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        completion_cb_ = std::move(cb);
    }

    ActionStatus NavigationClient::status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

    bool NavigationClient::sendGoal(const geometry_msgs::msg::PoseStamped &goal)
    {
        CompletionCallback cb;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (status_ == ActionStatus::PENDING)
            {
                return false;
            }
            if (skip_navigation_)
            {
                status_ = ActionStatus::SUCCEEDED;
                cb = completion_cb_;
            }
            else
            {
                status_ = ActionStatus::PENDING;
                pending_since_ = std::chrono::steady_clock::now();
                goal_pub_->publish(goal);
            }
        }
        if (cb)
        {
            cb(ActionStatus::SUCCEEDED, "navigation skipped");
        }
        return true;
    }

    void NavigationClient::reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = ActionStatus::IDLE;
        pending_since_.reset();
    }

    void NavigationClient::checkTimeout(std::chrono::milliseconds timeout)
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
            cb(ActionStatus::TIMEOUT, "navigation timeout");
        }
    }

    void NavigationClient::onArrived(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
        {
            return;
        }
        CompletionCallback cb;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (status_ != ActionStatus::PENDING)
            {
                return;
            }
            status_ = ActionStatus::SUCCEEDED;
            pending_since_.reset();
            cb = completion_cb_;
        }
        if (cb)
        {
            cb(ActionStatus::SUCCEEDED, "navigation arrived");
        }
    }

} // namespace coordination
