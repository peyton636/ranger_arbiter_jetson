/** @file motion_planner_node.cpp
 *  @brief MoveIt2 规划与执行监控：订阅规划请求，发布轨迹与执行反馈。
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <agx_motion_msgs/msg/execute_feedback.hpp>
#include <agx_motion_msgs/msg/execute_result.hpp>
#include <agx_motion_msgs/msg/motion_trajectory.hpp>
#include <agx_motion_msgs/msg/plan_request.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

//moveit + 任务调度器
namespace motion_planner_node
{

using agx_motion_msgs::msg::ExecuteFeedback;
using agx_motion_msgs::msg::ExecuteResult;
using agx_motion_msgs::msg::MotionTrajectory;
using agx_motion_msgs::msg::PlanRequest;

//PlanRequest
// JOINT      关节空间规划
// POSE       末端位姿规划
// NAMED      预设姿态
// CARTESIAN  直线运动

// move_group->plan(plan)
// computeCartesianPath(...)核心，但是只是算轨迹，不执行

class MotionPlannerNode : public rclcpp::Node
{
public:
  MotionPlannerNode()
  : Node("motion_planner_node")
  {
    declare_parameter<std::string>("default_arm_group", "arm");
    declare_parameter<std::string>("plan_request_topic", "/motion/plan_request");
    declare_parameter<std::string>("grasp_pose_topic", "/grasp/selected_pose");
    declare_parameter<std::string>("joint_states_topic", "/joint/states");
    declare_parameter<std::string>("trajectory_topic", "/motion/trajectory");
    declare_parameter<std::string>("execute_feedback_topic", "/motion/execute_feedback");
    declare_parameter<std::string>("execute_result_topic", "/motion/execute_result");
    declare_parameter<double>("execute_timeout_sec", 120.0);

    default_arm_group_ = get_parameter("default_arm_group").as_string();
    execute_timeout_sec_ = get_parameter("execute_timeout_sec").as_double();

    feedback_pub_ = create_publisher<ExecuteFeedback>(
      get_parameter("execute_feedback_topic").as_string(), 10);
    trajectory_pub_ = create_publisher<MotionTrajectory>(
      get_parameter("trajectory_topic").as_string(), 10);

    execute_result_sub_ = create_subscription<ExecuteResult>(
      get_parameter("execute_result_topic").as_string(), 10,
      [this](const ExecuteResult::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_execute_result_ = *msg;
        execute_result_cv_.notify_all();
      });

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_states_topic").as_string(), 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_joint_states_ = *msg;
      });

    grasp_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("grasp_pose_topic").as_string(), 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_grasp_pose_ = *msg;
      });

    plan_request_sub_ = create_subscription<PlanRequest>(
      get_parameter("plan_request_topic").as_string(), 10,
      [this](const PlanRequest::SharedPtr msg) {
        std::thread([this, msg]() { onPlanRequest(msg); }).detach();
      });

    RCLCPP_INFO(get_logger(), "motion_planner_node ready (default group: %s)", default_arm_group_.c_str());
  }

private:
  void publishFeedback(
    const std::string & request_id, uint8_t status, float progress, const std::string & message)
  {
    ExecuteFeedback msg;
    msg.header.stamp = now();
    msg.request_id = request_id;
    msg.status = status;
    msg.progress = progress;
    msg.message = message;
    feedback_pub_->publish(msg);
  }

  bool waitForExecuteResult(const std::string & request_id)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(execute_timeout_sec_);

    std::unique_lock<std::mutex> lock(mutex_);
    while (rclcpp::ok()) {
      if (last_execute_result_.has_value() &&
        last_execute_result_->request_id == request_id)
      {
        return last_execute_result_->success;
      }
      if (execute_result_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
        RCLCPP_ERROR(get_logger(), "execute timeout for request_id=%s", request_id.c_str());
        return false;
      }
    }
    return false;
  }

  void onPlanRequest(const PlanRequest::SharedPtr request)
  {
    const std::string request_id = request->request_id.empty() ?
      ("plan_" + std::to_string(now().nanoseconds())) : request->request_id;

    publishFeedback(request_id, ExecuteFeedback::STATUS_PLANNING, 0.0, "planning");

    const std::string group_name = request->group_name.empty() ?
      default_arm_group_ : request->group_name;

    try {
      auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), group_name);

      if (!request->planner_id.empty()) {
        move_group->setPlannerId(request->planner_id);
      }
      move_group->setMaxVelocityScalingFactor(
        request->max_velocity_scaling > 0.0 ? request->max_velocity_scaling : 1.0);
      move_group->setMaxAccelerationScalingFactor(
        request->max_acceleration_scaling > 0.0 ? request->max_acceleration_scaling : 1.0);
      move_group->setStartStateToCurrentState();

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_ok = false;

      switch (request->plan_type) {
        case PlanRequest::PLAN_TYPE_JOINT:
          if (!request->joint_goal.empty()) {
            move_group->setJointValueTarget(request->joint_goal);
            plan_ok = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
          }
          break;

        case PlanRequest::PLAN_TYPE_NAMED_TARGET:
          if (!request->named_target.empty()) {
            move_group->setNamedTarget(request->named_target);
            plan_ok = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
          }
          break;

        case PlanRequest::PLAN_TYPE_POSE: {
          geometry_msgs::msg::PoseStamped goal = request->pose_goal;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (latest_grasp_pose_.has_value() && request->pose_goal.header.frame_id.empty()) {
              goal = latest_grasp_pose_.value();
            }
          }
          move_group->setPoseTarget(goal);
          plan_ok = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
          break;
        }

        case PlanRequest::PLAN_TYPE_CARTESIAN: {
          std::vector<geometry_msgs::msg::Pose> waypoints;
          auto current = move_group->getCurrentPose();
          geometry_msgs::msg::Pose target = current.pose;
          const auto & dir = request->cartesian_direction.vector;
          const double dist = request->cartesian_max_dist > 0.0 ?
            request->cartesian_max_dist : 0.05;
          target.position.x += dir.x * dist;
          target.position.y += dir.y * dist;
          target.position.z += dir.z * dist;
          waypoints.push_back(target);

          moveit_msgs::msg::RobotTrajectory trajectory;
          const double step = request->cartesian_step_size > 0.0 ?
            request->cartesian_step_size : 0.01;
          const double fraction = move_group->computeCartesianPath(
            waypoints, step, request->cartesian_min_dist, trajectory);
          plan_ok = fraction >= 0.95;
          if (plan_ok) {
            plan.trajectory_ = trajectory;
          } else {
            RCLCPP_WARN(
              get_logger(), "cartesian path fraction=%.2f (<0.95)", fraction);
          }
          break;
        }

        default:
          publishFeedback(
            request_id, ExecuteFeedback::STATUS_FAILED, 0.0,
            "unsupported plan_type");
          return;
      }

      if (!plan_ok) {
        publishFeedback(request_id, ExecuteFeedback::STATUS_FAILED, 0.0, "planning failed");
        return;
      }

      MotionTrajectory traj_msg;
      traj_msg.header.stamp = now();
      traj_msg.request_id = request_id;
      traj_msg.trajectory = plan.trajectory_;
      trajectory_pub_->publish(traj_msg);//发布轨迹，让arm_controller_node执行/motion/trajectory
      publishFeedback(request_id, ExecuteFeedback::STATUS_PLANNED, 0.5, "trajectory published");

      if (!request->execute) {
        publishFeedback(request_id, ExecuteFeedback::STATUS_SUCCEEDED, 1.0, "plan only");
        return;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_execute_result_.reset();
      }

      publishFeedback(request_id, ExecuteFeedback::STATUS_EXECUTING, 0.6, "waiting arm_controller");
      const bool success = waitForExecuteResult(request_id);//等待arm_controller_node执行完成，他会监听/motion/execute_result
      publishFeedback(
        request_id,
        success ? ExecuteFeedback::STATUS_SUCCEEDED : ExecuteFeedback::STATUS_FAILED,
        success ? 1.0 : 0.0,
        success ? "execution succeeded" : "execution failed");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "plan request failed: %s", e.what());
      publishFeedback(request_id, ExecuteFeedback::STATUS_FAILED, 0.0, e.what());
    }
  }

  std::string default_arm_group_;
  double execute_timeout_sec_{120.0};

  rclcpp::Publisher<ExecuteFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<MotionTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<ExecuteResult>::SharedPtr execute_result_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_sub_;
  rclcpp::Subscription<PlanRequest>::SharedPtr plan_request_sub_;

  std::mutex mutex_;
  std::condition_variable execute_result_cv_;
  std::optional<ExecuteResult> last_execute_result_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_states_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_grasp_pose_;
};

}  // namespace motion_planner_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motion_planner_node::MotionPlannerNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
