#ifndef AGX_MOTION_PLANNER_NODE_HPP
#define AGX_MOTION_PLANNER_NODE_HPP

#include "visibility_control.hpp"
#include <agx_motion_msgs/msg/execute_feedback.hpp>
#include <agx_motion_msgs/msg/execute_result.hpp>
#include <agx_motion_msgs/msg/motion_trajectory.hpp>
#include <agx_motion_msgs/msg/plan_request.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace manipulation
{

    using agx_motion_msgs::msg::ExecuteFeedback;
    using agx_motion_msgs::msg::ExecuteResult;
    using agx_motion_msgs::msg::MotionTrajectory;
    using agx_motion_msgs::msg::PlanRequest;

    class MANIPULATION_AGX_MOTION_PLANNER_PUBLIC MotionPlannerNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit MotionPlannerNode(const rclcpp::NodeOptions &options);
        ~MotionPlannerNode() = default;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:
        void declareParameters();

        void onPlanRequest(const agx_motion_msgs::msg::PlanRequest::SharedPtr msg);

        void publishFeedback(const std::string &request_id, uint8_t status, float progress, const std::string &message);

        moveit_msgs::msg::RobotTrajectory planToPose(const geometry_msgs::msg::PoseStamped &target_pose);

        bool waitForExecuteResult(const std::string &request_id);

    private:
        std::string default_arm_group_;
        double execute_timeout_sec_{120.0};

        // rclcpp::Publisher<agx_motion_msgs::msg::ExecuteResult>::SharedPtr result_pub_;
        // rclcpp::Publisher<agx_motion_msgs::msg::ExecuteFeedback>::SharedPtr feedback_pub_;
        // rclcpp::Subscription<agx_motion_msgs::msg::PlanRequest>::SharedPtr plan_sub_;
        // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        // std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
        // std::mutex mutex_;
        // std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        // std::atomic<bool> busy_{false};

        rclcpp_lifecycle::LifecyclePublisher<ExecuteFeedback>::SharedPtr feedback_pub_;
        rclcpp_lifecycle::LifecyclePublisher<MotionTrajectory>::SharedPtr trajectory_pub_;
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

} // namespace manipulation

#endif // AGX_MOTION_PLANNER_NODE_HPP