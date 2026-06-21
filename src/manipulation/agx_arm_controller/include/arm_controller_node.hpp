#ifndef AGX_ARM_CONTROLLER_NODE_HPP
#define AGX_ARM_CONTROLLER_NODE_HPP

/** @file arm_controller_node.cpp
 *  @brief 轨迹下发执行与结果回传：订阅 /motion/trajectory，驱动 /control/joint_states。
 */

#include "visibility_control.hpp"
#include <agx_motion_msgs/msg/execute_result.hpp>
#include <agx_motion_msgs/msg/motion_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace manipulation
{
    using agx_motion_msgs::msg::ExecuteResult;
    using agx_motion_msgs::msg::MotionTrajectory;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    struct JointWaypoint
    {
        std::vector<std::string> names;
        std::vector<double> positions;
        std::vector<double> efforts;
    };

    class MANIPULATION_AGX_ARM_CONTROLLER_PUBLIC ArmControllerNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit ArmControllerNode(const rclcpp::NodeOptions &options);
        ~ArmControllerNode() override = default;


        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:
        void declareParameters();

        // 接收机械臂真实反馈，转发到/joint/states
        void onFeedbackJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);

        // 接收轨迹，执行轨迹
        void onTrajectory(const MotionTrajectory::SharedPtr msg);

        // ros2_control:调用FollowJointTrajectory action接口 --- IGNORE ---
        void executeTrajectoryViaAction(const MotionTrajectory &msg);

        // 真机：逐点下发 /control/joint_states 并等待 /feedback/joint_states 到位 --- IGNORE ---
        void executeTrajectoryViaJointStates(const MotionTrajectory &msg);

        // 从轨迹中取关节点,把标准轨迹转成内部的数据结构std::vector<JointWaypoint>
        static std::vector<JointWaypoint> extractJointPoints(const MotionTrajectory & msg);

        // 判断机械臂是否到位 --- IGNORE ---
        bool waitUntilReached(
            const std::vector<std::string> &names,
            const std::vector<double> &target_positions);

        // 发布执行结果 --- IGNORE ---
        void publishResult(
            const std::string &request_id, bool success, int32_t error_code, const std::string &message);


    private:
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr control_pub_;
        rclcpp_lifecycle::LifecyclePublisher<ExecuteResult>::SharedPtr result_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
        rclcpp::Subscription<MotionTrajectory>::SharedPtr traj_sub_;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

        std::mutex feedback_mutex_;
        std::optional<sensor_msgs::msg::JointState> latest_feedback_;

        std::mutex exec_mutex_;
        bool executing_ = false;

        double control_rate_hz_;
        double goal_tolerance_;
        double reach_timeout_sec_;
        double action_server_timeout_sec_;
        double action_result_timeout_sec_;
        bool use_ros2_control_action_;
    };
} // namespace manipulation
#endif // AGX_ARM_CONTROLLER_NODE_HPP