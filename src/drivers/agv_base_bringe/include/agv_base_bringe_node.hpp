#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <jetson_mcu_msgs/msg/blob_agv_motion.hpp>
#include <jetson_mcu_msgs/msg/time_sync_response.hpp>
#include <jetson_mcu_msgs/msg/v3_command.hpp>
#include <jetson_mcu_msgs/msg/v3_ext_status.hpp>
#include <jetson_mcu_msgs/msg/v3_status.hpp>
#include <scr_sensor/msg/agv_control.hpp>
#include <scr_sensor/msg/agv_feature_status.hpp>
#include <scr_sensor/msg/vehicle_data.hpp>

#include "jetson/jetson_protocol.hpp"
#include "visibility_control.hpp"

namespace agv_base_driver
{

/// @brief AGV 底盘桥接 Lifecycle 节点
///
/// 对外业务 Topic（规范层）：
/// - 订阅 /agv_control
/// - 发布 /Vehicle/VehicleData、/Function/FeatureStatusInfo、/odom
///
/// 对内协议 Topic（逐步仅调试）：
/// - /jetson_{link_type}/command、v3_status、blob/* …
class AGV_BASE_DRIVER_PUBLIC AgvBaseBringeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit AgvBaseBringeNode(const rclcpp::NodeOptions & options);
  ~AgvBaseBringeNode() override = default;

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declareParameters();

  void onAgvControl(const scr_sensor::msg::AgvControl::SharedPtr msg);
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onStatus(const jetson_mcu_msgs::msg::V3Status::SharedPtr msg);
  void onExtStatus(const jetson_mcu_msgs::msg::V3ExtStatus::SharedPtr msg);
  void onMotionBlob(const jetson_mcu_msgs::msg::BlobAgvMotion::SharedPtr msg);
  void onTimeSync(const jetson_mcu_msgs::msg::TimeSyncResponse::SharedPtr msg);

  struct ResolvedMotion {
    int16_t v_mm_s{0};
    int16_t omega_millirad_s{0};
    int16_t steer_millirad{0};
    uint8_t motion_model{0};
    std::string mode;
  };

  void applyMotionCommand(const MotionCommand & motion);
  void applyAgvControlFields(const scr_sensor::msg::AgvControl & msg);

  void publishCommand();
  void publishState();

  [[nodiscard]] bool inRecovery() const;
  void startRecovery();
  [[nodiscard]] ResolvedMotion resolveMotion();

  [[nodiscard]] std::optional<scr_sensor::msg::VehicleData> buildVehicleData() const;
  [[nodiscard]] std::optional<scr_sensor::msg::AgvFeatureStatus> buildFeatureStatus() const;
  void integrateOdom(int16_t v_mm_s, int16_t omega_millirad_s);

  [[nodiscard]] static double steadyNowSec();

  std::string link_type_;
  std::string jetson_prefix_;
  std::string agv_control_topic_;
  std::string cmd_vel_topic_;
  std::string feature_status_topic_;
  std::string command_topic_;
  std::string status_topic_;
  std::string ext_status_topic_;
  std::string motion_blob_topic_;
  std::string time_sync_topic_;
  std::string vehicle_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double cmd_rate_hz_{50.0};
  double status_timeout_s_{0.5};
  double cmd_timeout_s_{0.5};
  double max_linear_m_s_{0.8};
  double max_angular_rad_s_{1.0};
  double cruise_scale_{1.0};
  uint8_t mode_req_{1};
  bool strafe_jl_from_angular_{true};
  double strafe_speed_m_s_{0.3};
  int32_t sideways_steer_millirad_{1571};
  bool cmd_vel_compat_enable_{true};

  mutable std::mutex mutex_;

  jetson_mcu_msgs::msg::V3Status latest_status_;
  jetson_mcu_msgs::msg::V3ExtStatus latest_ext_;
  jetson_mcu_msgs::msg::BlobAgvMotion latest_motion_;
  bool has_status_{false};
  bool has_ext_{false};
  bool has_motion_blob_{false};
  double last_status_time_{0.0};

  int16_t pending_v_mm_s_{0};
  int16_t pending_omega_{0};
  int16_t pending_steer_{0};
  uint8_t pending_motion_model_{0};
  uint8_t pending_mode_req_{1};
  uint8_t pending_clear_error_{0};
  uint8_t light_enable_{1};
  uint8_t light_mode_{0};
  double last_cmd_time_{0.0};
  int16_t last_motion_v_mm_s_{0};
  int16_t last_motion_omega_{0};
  int16_t last_motion_steer_{0};
  double recover_until_{0.0};

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_theta_{0.0};
  std::optional<double> last_odom_time_;

  double offset_ms_{0.0};
  bool offset_valid_{false};

  rclcpp::Subscription<scr_sensor::msg::AgvControl>::SharedPtr agv_control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<jetson_mcu_msgs::msg::V3Status>::SharedPtr status_sub_;
  rclcpp::Subscription<jetson_mcu_msgs::msg::V3ExtStatus>::SharedPtr ext_sub_;
  rclcpp::Subscription<jetson_mcu_msgs::msg::BlobAgvMotion>::SharedPtr motion_blob_sub_;
  rclcpp::Subscription<jetson_mcu_msgs::msg::TimeSyncResponse>::SharedPtr time_sync_sub_;

  rclcpp::Publisher<jetson_mcu_msgs::msg::V3Command>::SharedPtr command_pub_;
  rclcpp::Publisher<scr_sensor::msg::VehicleData>::SharedPtr vehicle_pub_;
  rclcpp::Publisher<scr_sensor::msg::AgvFeatureStatus>::SharedPtr feature_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace agv_base_driver
