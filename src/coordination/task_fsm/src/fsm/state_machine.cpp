#include "fsm/state_machine.hpp"

#include <algorithm>

namespace coordination
{

    StateMachine::StateMachine(
        rclcpp_lifecycle::LifecycleNode &node,
        PickPlacePolicy policy,
        std::shared_ptr<MotionClient> motion,
        std::shared_ptr<GripperClient> gripper,
        std::shared_ptr<NavigationClient> navigation,
        std::shared_ptr<PerceptionHandler> perception)
        : node_(node),
          policy_(std::move(policy)),
          motion_(std::move(motion)),
          gripper_(std::move(gripper)),
          navigation_(std::move(navigation)),
          perception_(std::move(perception))
    {
        state_pub_ = node_.create_publisher<std_msgs::msg::String>("/task/fsm_state", 10);
        pick_nav_goal_ = policy_.makeNavigationGoal(1.0, 0.0, 0.0);
        place_nav_goal_ = policy_.makeNavigationGoal(0.0, 1.0, 1.57);
        bindClientCallbacks();
        publishState();
    }

    void StateMachine::bindClientCallbacks()
    {
        auto cb = [this](ActionStatus status, const std::string &message)
        {
            onSubSystemDone(status, message);
        };
        motion_->setCompletionCallback(cb);
        gripper_->setCompletionCallback(cb);
        navigation_->setCompletionCallback(cb);
    }

    void StateMachine::tick()
    {
        motion_->checkTimeout(policy_.config().motion_timeout);
        gripper_->checkTimeout(policy_.config().gripper_timeout);
        navigation_->checkTimeout(policy_.config().navigation_timeout);

        if (ctx_.pending_command == TaskCommand::CANCEL && ctx_.state != TaskFsmState::IDLE)
        {
            transitionTo(TaskFsmState::ERROR, "task cancelled");
            ctx_.pending_command = TaskCommand::NONE;
        }
        else if (ctx_.pending_command == TaskCommand::RESET)
        {
            motion_->reset();
            gripper_->reset();
            navigation_->reset();
            perception_->resetSelection();
            ctx_.resetForNewTask();
            ctx_.enterState(TaskFsmState::IDLE);
            ctx_.pending_command = TaskCommand::NONE;
            publishState();
            return;
        }

        switch (ctx_.state)
        {
        case TaskFsmState::IDLE:
            stepIdle();
            break;
        case TaskFsmState::NAVIGATE_TO_PICK:
            stepNavigateToPick();
            break;
        case TaskFsmState::WAIT_PERCEPTION:
            stepWaitPerception();
            break;
        case TaskFsmState::SELECT_TARGET:
            stepSelectTarget();
            break;
        case TaskFsmState::OPEN_GRIPPER:
            stepOpenGripper();
            break;
        case TaskFsmState::MOVE_GRASP_READY:
            stepMoveGraspReady();
            break;
        case TaskFsmState::MOVE_PRE_GRASP:
            stepMovePreGrasp();
            break;
        case TaskFsmState::APPROACH_CARTESIAN:
            stepApproachCartesian();
            break;
        case TaskFsmState::CLOSE_GRIPPER:
            stepCloseGripper();
            break;
        case TaskFsmState::RETREAT:
            stepRetreat();
            break;
        case TaskFsmState::NAVIGATE_TO_PLACE:
            stepNavigateToPlace();
            break;
        case TaskFsmState::PLACE_OPEN_GRIPPER:
            stepPlaceOpenGripper();
            break;
        case TaskFsmState::PLACE_RETREAT:
            stepPlaceRetreat();
            break;
        case TaskFsmState::GO_HOME:
            stepGoHome();
            break;
        case TaskFsmState::RETRY_RECOVERY:
            stepRetryRecovery();
            break;
        case TaskFsmState::ERROR:
            stepError();
            break;
        case TaskFsmState::COMPLETED:
            stepCompleted();
            break;
        }
    }

    void StateMachine::requestStartPickPlace()
    {
        if (ctx_.state != TaskFsmState::IDLE && ctx_.state != TaskFsmState::COMPLETED &&
            ctx_.state != TaskFsmState::ERROR)
        {
            RCLCPP_WARN(node_.get_logger(), "Reject start: FSM busy in %s", taskStateToString(ctx_.state).c_str());
            return;
        }
        ctx_.resetForNewTask();
        perception_->resetSelection();
        transitionTo(TaskFsmState::NAVIGATE_TO_PICK, "start pick-place task");
    }

    void StateMachine::requestCancel() { ctx_.pending_command = TaskCommand::CANCEL; }
    void StateMachine::requestReset() { ctx_.pending_command = TaskCommand::RESET; }

    TaskFsmState StateMachine::state() const { return ctx_.state; }
    const TaskContext &StateMachine::context() const { return ctx_; }

    void StateMachine::publishState()
    {
        std_msgs::msg::String msg;
        msg.data = taskStateToString(ctx_.state);
        state_pub_->publish(msg);
    }

    void StateMachine::transitionTo(TaskFsmState next, const std::string &reason)
    {
        RCLCPP_INFO(
            node_.get_logger(), "FSM %s -> %s (%s)",
            taskStateToString(ctx_.state).c_str(), taskStateToString(next).c_str(), reason.c_str());
        ctx_.enterState(next);
        publishState();
    }

    void StateMachine::failWith(const std::string &reason)
    {
        ctx_.last_error = reason;
        RCLCPP_ERROR(node_.get_logger(), "Task failed: %s", reason.c_str());
        transitionTo(TaskFsmState::ERROR, reason);
    }

    bool StateMachine::beginMotion(const agx_motion_msgs::msg::PlanRequest &req)
    {
        ctx_.motion_status = ActionStatus::PENDING;
        ctx_.active_request_id = req.request_id;
        return motion_->send(req);
    }

    bool StateMachine::beginGripper(const agx_motion_msgs::msg::GripperCmd &cmd)
    {
        ctx_.gripper_status = ActionStatus::PENDING;
        ctx_.active_request_id = cmd.request_id;
        return gripper_->send(cmd);
    }

    bool StateMachine::beginNavigation(const geometry_msgs::msg::PoseStamped &goal)
    {
        ctx_.navigation_status = ActionStatus::PENDING;
        return navigation_->sendGoal(goal);
    }

    void StateMachine::onSubSystemDone(ActionStatus status, const std::string &message)
    {
        if (status == ActionStatus::SUCCEEDED)
        {
            ctx_.motion_status = motion_->status();
            ctx_.gripper_status = gripper_->status();
            ctx_.navigation_status = navigation_->status();
            return;
        }

        ctx_.last_error = message.empty() ? actionStatusToString(status) : message;

        switch (ctx_.state)
        {
        case TaskFsmState::NAVIGATE_TO_PICK:
        case TaskFsmState::NAVIGATE_TO_PLACE:
            if (++ctx_.navigation_retry_count <= policy_.config().retry.max_navigation_retries)
            {
                transitionTo(TaskFsmState::RETRY_RECOVERY, "navigation failed, retry");
            }
            else
            {
                failWith("navigation failed after retries");
            }
            break;
        case TaskFsmState::WAIT_PERCEPTION:
            transitionTo(TaskFsmState::RETRY_RECOVERY, "perception timeout");
            break;
        case TaskFsmState::CLOSE_GRIPPER:
            if (++ctx_.gripper_retry_count <= policy_.config().retry.max_gripper_retries)
            {
                transitionTo(TaskFsmState::RETRY_RECOVERY, "gripper failed, retry");
            }
            else
            {
                failWith("gripper failed after retries");
            }
            break;
        default:
            if (++ctx_.motion_retry_count <= policy_.config().retry.max_motion_retries)
            {
                transitionTo(TaskFsmState::RETRY_RECOVERY, "motion failed, retry");
            }
            else
            {
                failWith("motion failed after retries");
            }
            break;
        }
    }

    void StateMachine::stepIdle()
    {
        if (ctx_.pending_command == TaskCommand::START_PICK_PLACE)
        {
            ctx_.pending_command = TaskCommand::NONE;
            requestStartPickPlace();
        }
    }

    void StateMachine::stepNavigateToPick()
    {
        if (ctx_.navigation_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.navigation_status == ActionStatus::IDLE)
        {
            if (!beginNavigation(pick_nav_goal_))
            {
                failWith("failed to send pick navigation goal");
            }
            return;
        }
        if (ctx_.navigation_status == ActionStatus::SUCCEEDED)
        {
            ctx_.navigation_status = ActionStatus::IDLE;
            navigation_->reset();
            ctx_.setDeadlineFromNow(policy_.config().perception_timeout);
            transitionTo(TaskFsmState::WAIT_PERCEPTION, "arrived pick station");
        }
    }

    void StateMachine::stepWaitPerception()
    {
        if (perception_->hasTargets())
        {
            transitionTo(TaskFsmState::SELECT_TARGET, "targets detected");
            return;
        }
        if (ctx_.isDeadlineExpired())
        {
            if (++ctx_.perception_retry_count <= policy_.config().retry.max_perception_retries)
            {
                transitionTo(TaskFsmState::RETRY_RECOVERY, "perception timeout");
            }
            else
            {
                failWith("perception timeout after retries");
            }
        }
    }

    void StateMachine::stepSelectTarget()
    {
        auto target = perception_->selectBestTarget();
        if (!target.has_value())
        {
            transitionTo(TaskFsmState::WAIT_PERCEPTION, "no valid target");
            ctx_.setDeadlineFromNow(policy_.config().perception_timeout);
            return;
        }
        ctx_.selected_grasp_pose = target;
        perception_->publishSelectedGraspPose(target.value());
        transitionTo(TaskFsmState::OPEN_GRIPPER, "target selected");
    }

    void StateMachine::stepOpenGripper()
    {
        if (ctx_.gripper_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.gripper_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_grip_open_" + std::to_string(++request_counter_);
            const auto cmd = policy_.makeGripperCmd(req_id, agx_motion_msgs::msg::GripperCmd::CMD_OPEN, false);
            if (!beginGripper(cmd))
            {
                failWith("failed to open gripper");
            }
            return;
        }
        if (ctx_.gripper_status == ActionStatus::SUCCEEDED)
        {
            ctx_.gripper_status = ActionStatus::IDLE;
            gripper_->reset();
            transitionTo(TaskFsmState::MOVE_GRASP_READY, "gripper opened");
        }
    }

    void StateMachine::stepMoveGraspReady()
    {
        if (ctx_.motion_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.motion_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_ready_" + std::to_string(++request_counter_);
            const auto req = policy_.makeNamedPlan(req_id, policy_.config().named_grasp_ready, true);
            if (!beginMotion(req))
            {
                failWith("failed to move grasp_ready");
            }
            return;
        }
        if (ctx_.motion_status == ActionStatus::SUCCEEDED)
        {
            ctx_.motion_status = ActionStatus::IDLE;
            motion_->reset();
            transitionTo(TaskFsmState::MOVE_PRE_GRASP, "at grasp_ready");
        }
    }

    void StateMachine::stepMovePreGrasp()
    {
        if (ctx_.motion_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.motion_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_pre_grasp_" + std::to_string(++request_counter_);
            agx_motion_msgs::msg::PlanRequest req;
            if (ctx_.selected_grasp_pose.has_value())
            {
                req = policy_.makePosePlan(req_id, ctx_.selected_grasp_pose.value(), true);
            }
            else
            {
                req = policy_.makeNamedPlan(req_id, policy_.config().named_pre_grasp, true);
            }
            if (!beginMotion(req))
            {
                failWith("failed to move pre_grasp");
            }
            return;
        }
        if (ctx_.motion_status == ActionStatus::SUCCEEDED)
        {
            ctx_.motion_status = ActionStatus::IDLE;
            motion_->reset();
            transitionTo(TaskFsmState::APPROACH_CARTESIAN, "at pre_grasp");
        }
    }

    void StateMachine::stepApproachCartesian()
    {
        if (ctx_.motion_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.motion_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_approach_" + std::to_string(++request_counter_);
            const auto req = policy_.makeCartesianPlan(
                req_id, policy_.makeTcpDownDirection(), policy_.config().approach_dist_m, true);
            if (!beginMotion(req))
            {
                failWith("failed cartesian approach");
            }
            return;
        }
        if (ctx_.motion_status == ActionStatus::SUCCEEDED)
        {
            ctx_.motion_status = ActionStatus::IDLE;
            motion_->reset();
            transitionTo(TaskFsmState::CLOSE_GRIPPER, "approach done");
        }
    }

    void StateMachine::stepCloseGripper()
    {
        if (ctx_.gripper_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.gripper_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_grip_close_" + std::to_string(++request_counter_);
            const auto cmd = policy_.makeGripperCmd(req_id, agx_motion_msgs::msg::GripperCmd::CMD_CLOSE, true);
            if (!beginGripper(cmd))
            {
                failWith("failed to close gripper");
            }
            return;
        }
        if (ctx_.gripper_status == ActionStatus::SUCCEEDED)
        {
            ctx_.gripper_status = ActionStatus::IDLE;
            gripper_->reset();
            transitionTo(TaskFsmState::RETREAT, "object grasped");
        }
    }

    void StateMachine::stepRetreat()
    {
        if (ctx_.motion_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.motion_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_retreat_" + std::to_string(++request_counter_);
            const auto req = policy_.makeNamedPlan(req_id, policy_.config().named_retreat, true);
            if (!beginMotion(req))
            {
                failWith("failed retreat");
            }
            return;
        }
        if (ctx_.motion_status == ActionStatus::SUCCEEDED)
        {
            ctx_.motion_status = ActionStatus::IDLE;
            motion_->reset();
            transitionTo(TaskFsmState::NAVIGATE_TO_PLACE, "retreat done");
        }
    }

    void StateMachine::stepNavigateToPlace()
    {
        if (ctx_.navigation_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.navigation_status == ActionStatus::IDLE)
        {
            if (!beginNavigation(place_nav_goal_))
            {
                failWith("failed to send place navigation goal");
            }
            return;
        }
        if (ctx_.navigation_status == ActionStatus::SUCCEEDED)
        {
            ctx_.navigation_status = ActionStatus::IDLE;
            navigation_->reset();
            transitionTo(TaskFsmState::PLACE_OPEN_GRIPPER, "arrived place station");
        }
    }

    void StateMachine::stepPlaceOpenGripper()
    {
        if (ctx_.gripper_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.gripper_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_place_open_" + std::to_string(++request_counter_);
            const auto cmd = policy_.makeGripperCmd(req_id, agx_motion_msgs::msg::GripperCmd::CMD_OPEN, false);
            if (!beginGripper(cmd))
            {
                failWith("failed to open gripper at place");
            }
            return;
        }
        if (ctx_.gripper_status == ActionStatus::SUCCEEDED)
        {
            ctx_.gripper_status = ActionStatus::IDLE;
            gripper_->reset();
            transitionTo(TaskFsmState::PLACE_RETREAT, "object released");
        }
    }

    void StateMachine::stepPlaceRetreat()
    {
        if (ctx_.motion_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.motion_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_place_retreat_" + std::to_string(++request_counter_);
            const auto req = policy_.makeCartesianPlan(
                req_id, policy_.makeTcpUpDirection(), policy_.config().place_retreat_dist_m, true);
            if (!beginMotion(req))
            {
                failWith("failed place retreat");
            }
            return;
        }
        if (ctx_.motion_status == ActionStatus::SUCCEEDED)
        {
            ctx_.motion_status = ActionStatus::IDLE;
            motion_->reset();
            transitionTo(TaskFsmState::GO_HOME, "place retreat done");
        }
    }

    void StateMachine::stepGoHome()
    {
        if (ctx_.motion_status == ActionStatus::PENDING)
        {
            return;
        }
        if (ctx_.motion_status == ActionStatus::IDLE)
        {
            const auto req_id = "task_home_" + std::to_string(++request_counter_);
            const auto req = policy_.makeNamedPlan(req_id, policy_.config().named_home, true);
            if (!beginMotion(req))
            {
                failWith("failed go home");
            }
            return;
        }
        if (ctx_.motion_status == ActionStatus::SUCCEEDED)
        {
            ctx_.motion_status = ActionStatus::IDLE;
            motion_->reset();
            transitionTo(TaskFsmState::COMPLETED, "task completed");
        }
    }

    void StateMachine::stepRetryRecovery()
    {
        motion_->reset();
        gripper_->reset();
        navigation_->reset();
        ctx_.motion_status = ActionStatus::IDLE;
        ctx_.gripper_status = ActionStatus::IDLE;
        ctx_.navigation_status = ActionStatus::IDLE;

        if (ctx_.perception_retry_count > 0 && !perception_->hasTargets())
        {
            ctx_.setDeadlineFromNow(policy_.config().perception_timeout);
            transitionTo(TaskFsmState::WAIT_PERCEPTION, "retry perception");
            return;
        }
        if (ctx_.navigation_retry_count > 0)
        {
            transitionTo(
                ctx_.navigation_retry_count % 2 == 1 ? TaskFsmState::NAVIGATE_TO_PICK : TaskFsmState::NAVIGATE_TO_PLACE,
                "retry navigation");
            return;
        }
        transitionTo(TaskFsmState::MOVE_GRASP_READY, "retry motion from safe point");
    }

    void StateMachine::stepError()
    {
        motion_->reset();
        gripper_->reset();
        navigation_->reset();
    }

    void StateMachine::stepCompleted()
    {
        transitionTo(TaskFsmState::IDLE, "ready for next task");
    }

} // namespace coordination
