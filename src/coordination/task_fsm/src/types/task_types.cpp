#include "types/task_types.hpp"

namespace coordination
{

    std::string taskStateToString(TaskFsmState state)
    {
        switch (state)
        {
        case TaskFsmState::IDLE:
            return "IDLE";
        case TaskFsmState::NAVIGATE_TO_PICK:
            return "NAVIGATE_TO_PICK";
        case TaskFsmState::WAIT_PERCEPTION:
            return "WAIT_PERCEPTION";
        case TaskFsmState::SELECT_TARGET:
            return "SELECT_TARGET";
        case TaskFsmState::OPEN_GRIPPER:
            return "OPEN_GRIPPER";
        case TaskFsmState::MOVE_GRASP_READY:
            return "MOVE_GRASP_READY";
        case TaskFsmState::MOVE_PRE_GRASP:
            return "MOVE_PRE_GRASP";
        case TaskFsmState::APPROACH_CARTESIAN:
            return "APPROACH_CARTESIAN";
        case TaskFsmState::CLOSE_GRIPPER:
            return "CLOSE_GRIPPER";
        case TaskFsmState::RETREAT:
            return "RETREAT";
        case TaskFsmState::NAVIGATE_TO_PLACE:
            return "NAVIGATE_TO_PLACE";
        case TaskFsmState::PLACE_OPEN_GRIPPER:
            return "PLACE_OPEN_GRIPPER";
        case TaskFsmState::PLACE_RETREAT:
            return "PLACE_RETREAT";
        case TaskFsmState::GO_HOME:
            return "GO_HOME";
        case TaskFsmState::RETRY_RECOVERY:
            return "RETRY_RECOVERY";
        case TaskFsmState::ERROR:
            return "ERROR";
        case TaskFsmState::COMPLETED:
            return "COMPLETED";
        default:
            return "UNKNOWN";
        }
    }

    std::string actionStatusToString(ActionStatus status)
    {
        switch (status)
        {
        case ActionStatus::IDLE:
            return "IDLE";
        case ActionStatus::PENDING:
            return "PENDING";
        case ActionStatus::SUCCEEDED:
            return "SUCCEEDED";
        case ActionStatus::FAILED:
            return "FAILED";
        case ActionStatus::TIMEOUT:
            return "TIMEOUT";
        default:
            return "UNKNOWN";
        }
    }

} // namespace coordination
