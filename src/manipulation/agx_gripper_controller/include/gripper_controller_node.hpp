#ifndef GRIPPER_CONTROLLER_NODE_HPP
#define GRIPPER_CONTROLLER_NODE_HPP


#include "visibility_control.hpp"
#include <agx_arm_msgs/msg/gripper_status.hpp>
#include <agx_motion_msgs/msg/gripper_cmd.hpp>
#include <agx_motion_msgs/msg/gripper_control_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>


namespace manipulation
{
    using sensor_msgs::msg::JointState;
    using agx_arm_msgs::msg::GripperStatus;
    using agx_motion_msgs::msg::GripperCmd;
    using agx_motion_msgs::msg::GripperControlStatus;

    constexpr const char * kGripperJointName = "gripper";
    constexpr double kWidthMin = 0.0;
    constexpr double kWidthMax = 0.1;
    constexpr double kForceMin = 0.5;
    constexpr double kForceMax = 3.0;

class MANIPULATION_AGX_GRIPPER_CONTROLLER_PUBLIC GripperControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit GripperControllerNode(const rclcpp::NodeOptions &options);
    ~GripperControllerNode() override = default;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
    void declareParameters();

    static double clampWidth(double width);

    static double clampForce(double force);

    void executeCmd(const GripperCmd & msg);

    void sendGripperCmd(double width, double force) const;

    bool isWidthReached(double current_width, double target_width, bool opening) const;

    // 与 agx_arm_ctrl 一致：周期性向 /control/joint_states 下发 gripper 指令，直到到位或超时
    bool waitWidth(double target_width, double target_force, bool opening) const;

    bool waitGrasp(
        double target_width, double target_force,
        bool &force_ok, bool &current_ok) const;

    // 接收夹爪命令，执行命令
    void onGripperCmd(const agx_motion_msgs::msg::GripperCmd::SharedPtr msg);

    // 接收夹爪真实反馈，转发到/joint/states
    void onFeedbackJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);

    // 发布执行结果
    void publishResult(
        const std::string &request_id, bool success, int32_t error_code, const std::string &message);

    void publishStatus(const std::string & request_id, uint8_t state, 
            bool force_threshold_met, bool current_threshold_met, const std::string & message);

private:
    rclcpp_lifecycle::LifecyclePublisher<GripperControlStatus>::SharedPtr status_pub_;
    rclcpp_lifecycle::LifecyclePublisher<JointState>::SharedPtr control_pub_;

    // 关键修复：订阅器必须是 Subscription 类型
    rclcpp::Subscription<GripperStatus>::SharedPtr raw_sub_;
    rclcpp::Subscription<GripperCmd>::SharedPtr cmd_sub_;

    std::atomic_bool busy_{false};
    mutable std::mutex mutex_;
    std::optional<GripperStatus> latest_raw_;

    double open_width_{0.07};
    double close_width_{0.0};
    double default_force_{1.5};
    double force_grasp_threshold_{0.8};
    double current_grasp_threshold_{0.5};
    double grasp_timeout_sec_{8.0};
    double width_tolerance_{0.003};
    double control_rate_hz_{20.0};


    //   double open_width_{0.07};
//   double close_width_{0.0};
//   double default_force_{1.5};
//   double force_grasp_threshold_{0.8};
//   double current_grasp_threshold_{0.5};
//   double grasp_timeout_sec_{8.0};
//   double width_tolerance_{0.003};
//   double control_rate_hz_{20.0};

//   rclcpp::Publisher<GripperControlStatus>::SharedPtr status_pub_;
//   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr control_pub_;
//   rclcpp::Subscription<GripperStatus>::SharedPtr raw_sub_;
//   rclcpp::Subscription<GripperCmd>::SharedPtr cmd_sub_;

//   mutable std::mutex mutex_;
//   std::optional<GripperStatus> latest_raw_;
//   std::atomic_bool busy_{false};
};
} // namespace manipulation


#endif  // GRIPPER_CONTROLLER_NODE_HPP