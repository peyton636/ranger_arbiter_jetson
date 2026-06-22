#include "policy/pick_place_policy.hpp"

#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace coordination
{

    PickPlacePolicy::PickPlacePolicy(PickPlacePolicyConfig config)
        : config_(std::move(config))
    {
    }

    agx_motion_msgs::msg::PlanRequest PickPlacePolicy::makeNamedPlan(
        const std::string &request_id,
        const std::string &named_target,
        bool execute) const
    {
        agx_motion_msgs::msg::PlanRequest req;
        req.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        req.plan_type = agx_motion_msgs::msg::PlanRequest::PLAN_TYPE_NAMED_TARGET;
        req.request_id = request_id;
        req.group_name = config_.arm_group;
        req.named_target = named_target;
        req.execute = execute;
        req.max_velocity_scaling = config_.velocity_scaling;
        req.max_acceleration_scaling = config_.acceleration_scaling;
        return req;
    }

    agx_motion_msgs::msg::PlanRequest PickPlacePolicy::makePosePlan(
        const std::string &request_id,
        const geometry_msgs::msg::PoseStamped &pose,
        bool execute) const
    {
        agx_motion_msgs::msg::PlanRequest req;
        req.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        req.plan_type = agx_motion_msgs::msg::PlanRequest::PLAN_TYPE_POSE;
        req.request_id = request_id;
        req.group_name = config_.arm_group;
        req.pose_goal = pose;
        req.execute = execute;
        req.max_velocity_scaling = config_.velocity_scaling;
        req.max_acceleration_scaling = config_.acceleration_scaling;
        return req;
    }

    agx_motion_msgs::msg::PlanRequest PickPlacePolicy::makeCartesianPlan(
        const std::string &request_id,
        const geometry_msgs::msg::Vector3Stamped &direction,
        double distance_m,
        bool execute) const
    {
        agx_motion_msgs::msg::PlanRequest req;
        req.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        req.plan_type = agx_motion_msgs::msg::PlanRequest::PLAN_TYPE_CARTESIAN;
        req.request_id = request_id;
        req.group_name = config_.arm_group;
        req.cartesian_direction = direction;
        req.cartesian_min_dist = 0.0;
        req.cartesian_max_dist = distance_m;
        req.cartesian_step_size = config_.approach_step_m;
        req.execute = execute;
        req.max_velocity_scaling = config_.velocity_scaling;
        req.max_acceleration_scaling = config_.acceleration_scaling;
        return req;
    }

    agx_motion_msgs::msg::GripperCmd PickPlacePolicy::makeGripperCmd(
        const std::string &request_id,
        uint8_t command,
        bool wait_grasp) const
    {
        agx_motion_msgs::msg::GripperCmd cmd;
        cmd.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        cmd.request_id = request_id;
        cmd.command = command;
        cmd.target_width = config_.gripper_open_width;
        cmd.max_force = config_.gripper_close_force;
        cmd.wait_grasp = wait_grasp;
        return cmd;
    }

    geometry_msgs::msg::Vector3Stamped PickPlacePolicy::makeTcpDownDirection() const
    {
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = "tcp_link";
        dir.vector.x = 0.0;
        dir.vector.y = 0.0;
        dir.vector.z = -1.0;
        return dir;
    }

    geometry_msgs::msg::Vector3Stamped PickPlacePolicy::makeTcpUpDirection() const
    {
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = "tcp_link";
        dir.vector.x = 0.0;
        dir.vector.y = 0.0;
        dir.vector.z = 1.0;
        return dir;
    }

    geometry_msgs::msg::PoseStamped PickPlacePolicy::makeNavigationGoal(
        double x, double y, double yaw) const
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = config_.frame_id;
        goal.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.z = std::sin(yaw * 0.5);
        goal.pose.orientation.w = std::cos(yaw * 0.5);
        return goal;
    }

} // namespace coordination
