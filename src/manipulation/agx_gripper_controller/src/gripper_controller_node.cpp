/** @file gripper_controller_node.cpp
 *  @brief 夹爪闭环控制：力阈值与电流阈值双判据。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <agx_arm_msgs/msg/gripper_status.hpp>
#include <agx_motion_msgs/msg/gripper_cmd.hpp>
#include <agx_motion_msgs/msg/gripper_control_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace gripper_controller_node
{

using agx_arm_msgs::msg::GripperStatus;
using agx_motion_msgs::msg::GripperCmd;
using agx_motion_msgs::msg::GripperControlStatus;

constexpr const char * kGripperJointName = "gripper";
constexpr double kWidthMin = 0.0;
constexpr double kWidthMax = 0.1;
constexpr double kForceMin = 0.5;
constexpr double kForceMax = 3.0;

class GripperControllerNode : public rclcpp::Node
{
public:
  GripperControllerNode()
  : Node("gripper_controller_node")
  {
    declare_parameter<std::string>("gripper_cmd_topic", "/task/gripper_cmd");
    declare_parameter<std::string>("raw_status_topic", "/gripper/raw_status");
    declare_parameter<std::string>("status_topic", "/gripper/status");
    declare_parameter<std::string>(
      "control_joint_states_topic", "/control/gripper_joint_states");
    declare_parameter<double>("open_width", 0.07);
    declare_parameter<double>("close_width", 0.0);
    declare_parameter<double>("default_force", 1.5);
    declare_parameter<double>("force_grasp_threshold", 0.8);
    declare_parameter<double>("current_grasp_threshold", 0.5);
    declare_parameter<double>("grasp_timeout_sec", 8.0);
    declare_parameter<double>("width_tolerance", 0.003);
    declare_parameter<double>("control_rate_hz", 20.0);

    open_width_ = get_parameter("open_width").as_double();
    close_width_ = get_parameter("close_width").as_double();
    default_force_ = get_parameter("default_force").as_double();
    force_grasp_threshold_ = get_parameter("force_grasp_threshold").as_double();
    current_grasp_threshold_ = get_parameter("current_grasp_threshold").as_double();
    grasp_timeout_sec_ = get_parameter("grasp_timeout_sec").as_double();
    width_tolerance_ = get_parameter("width_tolerance").as_double();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();

    status_pub_ = create_publisher<GripperControlStatus>(
      get_parameter("status_topic").as_string(), 10);
    control_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("control_joint_states_topic").as_string(), 10);

    raw_sub_ = create_subscription<GripperStatus>(
      get_parameter("raw_status_topic").as_string(), 10,
      [this](const GripperStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_raw_ = *msg;
      });

    cmd_sub_ = create_subscription<GripperCmd>(
      get_parameter("gripper_cmd_topic").as_string(), 10,
      [this](const GripperCmd::SharedPtr msg) {
        onGripperCmd(msg);
      });

    RCLCPP_INFO(get_logger(), "gripper_controller_node ready");
  }

private:
  static double clampWidth(double width)
  {
    return std::max(kWidthMin, std::min(kWidthMax, std::abs(width)));
  }

  static double clampForce(double force)
  {
    return std::max(kForceMin, std::min(kForceMax, force));
  }

  void onGripperCmd(const GripperCmd::SharedPtr msg)
  {
    if (busy_.exchange(true)) {
      publishStatus(
        msg->request_id, GripperControlStatus::STATE_FAILED,
        false, false, "busy: previous command running");
      return;
    }

    std::thread([this, msg]() {
      executeCmd(*msg);
      busy_ = false;
    }).detach();
  }

  void executeCmd(const GripperCmd & msg)
  {
    std::string request_id = msg.request_id;
    if (request_id.empty()) {
      request_id = "gripper_" + std::to_string(now().nanoseconds());
    }

    if (msg.command == GripperCmd::CMD_STOP) {
      publishStatus(request_id, GripperControlStatus::STATE_IDLE, false, false, "stopped");
      return;
    }

    double target_width = msg.target_width;
    const double target_force = clampForce(msg.max_force > 0.0 ? msg.max_force : default_force_);

    if (msg.command == GripperCmd::CMD_OPEN) {
      if (target_width <= 0.0) {
        target_width = open_width_;
      }
      target_width = clampWidth(target_width);
      publishStatus(request_id, GripperControlStatus::STATE_MOVING, false, false, "opening");
      if (!waitWidth(target_width, target_force, true)) {
        publishStatus(request_id, GripperControlStatus::STATE_FAILED, false, false, "open timeout");
        return;
      }
      publishStatus(request_id, GripperControlStatus::STATE_OPEN, false, false, "opened");
      return;
    }

    if (msg.command == GripperCmd::CMD_CLOSE) {
      if (target_width < 0.0) {
        target_width = close_width_;
      }
      target_width = clampWidth(target_width);
      publishStatus(request_id, GripperControlStatus::STATE_MOVING, false, false, "closing");

      if (msg.wait_grasp) {
        bool force_ok = false;
        bool current_ok = false;
        const bool grasped = waitGrasp(target_width, target_force, force_ok, current_ok);
        if (grasped) {
          publishStatus(
            request_id, GripperControlStatus::STATE_GRASPED,
            force_ok, current_ok, "grasped");
        } else {
          publishStatus(
            request_id, GripperControlStatus::STATE_FAILED,
            force_ok, current_ok, "grasp timeout");
        }
        return;
      }

      if (!waitWidth(target_width, target_force, false)) {
        publishStatus(request_id, GripperControlStatus::STATE_FAILED, false, false, "close timeout");
        return;
      }
      publishStatus(request_id, GripperControlStatus::STATE_IDLE, false, false, "close sent");
    }
  }

  void sendGripperCmd(double width, double force) const
  {
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now();
    cmd.name = {kGripperJointName};
    cmd.position = {clampWidth(width)};
    cmd.effort = {clampForce(force)};
    control_pub_->publish(cmd);
  }

  bool isWidthReached(double current_width, double target_width, bool opening) const
  {
    if (opening) {
      return current_width >= target_width - width_tolerance_;
    }
    return current_width <= target_width + width_tolerance_;
  }

  // 与 agx_arm_ctrl 一致：周期性向 /control/joint_states 下发 gripper 指令，直到到位或超时
  bool waitWidth(double target_width, double target_force, bool opening) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(grasp_timeout_sec_);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0)));

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      sendGripperCmd(target_width, target_force);

      std::optional<GripperStatus> raw;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        raw = latest_raw_;
      }
      if (raw.has_value() && isWidthReached(raw->width, target_width, opening)) {
        return true;
      }
      std::this_thread::sleep_for(period);
    }
    return false;
  }

  bool waitGrasp(
    double target_width, double target_force,
    bool & force_ok, bool & current_ok) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(grasp_timeout_sec_);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0)));

    force_ok = false;
    current_ok = false;

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      sendGripperCmd(target_width, target_force);

      std::optional<GripperStatus> raw;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        raw = latest_raw_;
      }
      if (!raw.has_value()) {
        std::this_thread::sleep_for(period);
        continue;
      }

      force_ok = raw->force >= force_grasp_threshold_;
      current_ok = raw->driver_overcurrent || raw->force >= current_grasp_threshold_;
      if (force_ok || current_ok) {
        return true;
      }
      std::this_thread::sleep_for(period);
    }
    return false;
  }

  void publishStatus(
    const std::string & request_id,
    uint8_t state,
    bool force_threshold_met,
    bool current_threshold_met,
    const std::string & message)
  {
    GripperControlStatus status;
    status.header.stamp = now();
    status.request_id = request_id;
    status.state = state;
    status.force_threshold_met = force_threshold_met;
    status.current_threshold_met = current_threshold_met;
    status.message = message;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (latest_raw_.has_value()) {
        status.width = latest_raw_->width;
        status.force = latest_raw_->force;
      }
    }

    status_pub_->publish(status);
  }

  double open_width_{0.07};
  double close_width_{0.0};
  double default_force_{1.5};
  double force_grasp_threshold_{0.8};
  double current_grasp_threshold_{0.5};
  double grasp_timeout_sec_{8.0};
  double width_tolerance_{0.003};
  double control_rate_hz_{20.0};

  rclcpp::Publisher<GripperControlStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr control_pub_;
  rclcpp::Subscription<GripperStatus>::SharedPtr raw_sub_;
  rclcpp::Subscription<GripperCmd>::SharedPtr cmd_sub_;

  mutable std::mutex mutex_;
  std::optional<GripperStatus> latest_raw_;
  std::atomic_bool busy_{false};
};

}  // namespace gripper_controller_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gripper_controller_node::GripperControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
