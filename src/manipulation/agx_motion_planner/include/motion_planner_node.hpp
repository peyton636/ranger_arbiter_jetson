#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "visibility_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <agx_motion_msgs/msg/plan_request.hpp>
#include <agx_motion_msgs/msg/motion_trajectory.hpp>
#include <agx_motion_msgs/msg/execute_feedback.hpp>
#include <agx_motion_msgs/msg/execute_result.hpp>



namespace manipulation
{
class MANIPULATION_AGX_MOTION_PLANNER_PUBLIC MotionPlannerNode:public rclcpp_lifecycle::LifecycleNode
{
    public:
      explicit MotionPlannerNode(const rclcpp::NodeOptions&options=rclcpp::NodeOptions());

      using CallbackReturn=
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

      using PlanRequest = agx_motion_msgs::msg::PlanRequest;
      using MotionTrajectory = agx_motion_msgs::msg::MotionTrajectory;
      using ExecuteFeedback = agx_motion_msgs::msg::ExecuteFeedback;
      using ExecuteResult = agx_motion_msgs::msg::ExecuteResult;

      //Lifecycle 回调函数
      CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
      CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
      CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
      CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
      CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    private:
      
      //初始化参数
      void declareParameters();

      //规划请求入口
      void onPlanRequest(const PlanRequest::SharedPtr msg);

      //四种规划类型
      bool planJointTarget(
        moveit::planning_interface::MoveGroupInterface & move_group,
        const PlanRequest & request,
        moveit::planning_interface::MoveGroupInterface::Plan & plan);

      bool planNamedTarget(
        moveit::planning_interface::MoveGroupInterface & move_group,
        const PlanRequest & request,
        moveit::planning_interface::MoveGroupInterface::Plan & plan);

      bool planPoseTarget(
        moveit::planning_interface::MoveGroupInterface & move_group,
        const PlanRequest & request,
        moveit::planning_interface::MoveGroupInterface::Plan & plan,
        const std::string & request_id);

      bool planCartesianPath(
        moveit::planning_interface::MoveGroupInterface & move_group,
        const PlanRequest & request,
        moveit::planning_interface::MoveGroupInterface::Plan & plan);

      //工具函数
      void publishFeedback(
        const std::string & request_id,
        uint8_t status,
        float progress,
        const std::string & message);

      bool waitForExecuteResult(const std::string & reques_id);

    private:
      //参数
      std::string default_arm_group_;
      double execute_timeout_sec_{120.0};

      //Publisher
      rclcpp_lifecycle::LifecyclePublisher<ExecuteFeedback>::SharedPtr feedback_pub_;
      rclcpp_lifecycle::LifecyclePublisher<MotionTrajectory>::SharedPtr trajectory_pub_;

      //Subscriber
      rclcpp::Subscription<PlanRequest>::SharedPtr plan_request_sub_;
      rclcpp::Subscription<ExecuteResult>::SharedPtr execute_result_sub_;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_sub_;

      //缓存数据
      std::mutex mutex_;
      std::condition_variable execute_result_cv_;

      std::optional<ExecuteResult> last_execute_result_;
      std::optional<sensor_msgs::msg::JointState> latest_joint_states_;
      std::optional<geometry_msgs::msg::PoseStamped> latest_grasp_pose_;     
      
      //防止多个规划请求同时执行
      std::atomic_bool planning_busy_{false};
};
}