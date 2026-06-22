#include "types/task_context.hpp"

namespace coordination
{

    void TaskContext::resetForNewTask()
    {
        motion_status = ActionStatus::IDLE;
        gripper_status = ActionStatus::IDLE;
        navigation_status = ActionStatus::IDLE;
        active_request_id.clear();
        last_error.clear();
        perception_retry_count = 0;
        motion_retry_count = 0;
        gripper_retry_count = 0;
        navigation_retry_count = 0;
        selected_grasp_pose.reset();
        wait_deadline.reset();
    }

    void TaskContext::enterState(TaskFsmState next)
    {
        state = next;
        state_enter_time = std::chrono::steady_clock::now();
        wait_deadline.reset();
    }

    void TaskContext::setDeadlineFromNow(std::chrono::milliseconds timeout)
    {
        wait_deadline = std::chrono::steady_clock::now() + timeout;
    }

    bool TaskContext::isDeadlineExpired() const
    {
        if (!wait_deadline.has_value())
        {
            return false;
        }
        return std::chrono::steady_clock::now() >= wait_deadline.value();
    }

} // namespace coordination
