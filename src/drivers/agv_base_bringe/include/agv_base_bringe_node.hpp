#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <jetson_mcu_msgs/msg/time_sync_response.hpp>
#include <jetson_mcu_msgs/msg/v3_command.hpp>
#include <jetson_mcu_msgs/msg/v3_ext_status.hpp>
#include <jetson_mcu_msgs/msg/v3_status.hpp>
#include <scr_sensor/msg/vehicle_data.hpp>

#include "visibility_control.hpp"

namespace agv_base_driver
{

/// @brief AGV 底盘桥接 Lifecycle 节点
///
/// 职责：在 ROS 导航层与 STM32 V3 协议之间做双向适配。
/// - 下行：/cmd_vel → /jetson_{link_type}/command (V3Command)
/// - 上行：V3Status + V3ExtStatus → /vehicle/vehicle_data、/odom、odom→base_link TF
/// - 安全：cmd_vel 超时后强制 1s 零速恢复，避免 Nav2/上层断连时底盘继续运动
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

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onLightCmd(const std_msgs::msg::Bool::SharedPtr msg);
  void onStatus(const jetson_mcu_msgs::msg::V3Status::SharedPtr msg);
  void onExtStatus(const jetson_mcu_msgs::msg::V3ExtStatus::SharedPtr msg);
  void onTimeSync(const jetson_mcu_msgs::msg::TimeSyncResponse::SharedPtr msg);

  /// 定时下发 V3Command（含超时恢复逻辑）
  void publishCommand();
  /// 定时发布 VehicleData、/odom 与 TF
  void publishState();

  [[nodiscard]] bool inRecovery() const;
  void startRecovery();

  /// resolveMotion 的输出：经超时/恢复门控后的实际下发运动
  struct ResolvedMotion {
    int16_t v_mm_s{0};
    int16_t omega_millirad_s{0};
    int16_t steer_millirad{0};
    uint8_t motion_model{0};
    std::string mode;  // "RUN" | "IDLE" | "RECOVER"
  };
  /// 结合 cmd_vel 超时与恢复窗口，决定当前周期是否允许运动
  ResolvedMotion resolveMotion();

  /// 融合 V3Status + V3ExtStatus 为上层统一的 VehicleData
  [[nodiscard]] std::optional<scr_sensor::msg::VehicleData> buildVehicleData() const;
  /// 用底盘反馈速度做简易里程计积分（仅平移 + 自旋）
  void integrateOdom(int16_t v_mm_s, int16_t omega_millirad_s);

  /// 单调时钟秒，用于超时判断（不受 use_sim_time 影响）
  [[nodiscard]] static double steadyNowSec();

  // --- 话题与坐标系 ---
  std::string link_type_;       // rs232 | eth | can，决定 /jetson_{link_type} 前缀
  std::string jetson_prefix_;
  std::string cmd_vel_topic_;
  std::string light_cmd_topic_;
  std::string command_topic_;
  std::string status_topic_;
  std::string ext_status_topic_;
  std::string time_sync_topic_;
  std::string vehicle_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  // --- 控制参数 ---
  double cmd_rate_hz_{50.0};
  double status_timeout_s_{0.5};
  double cmd_timeout_s_{0.5};
  double max_linear_m_s_{0.8};
  double max_angular_rad_s_{1.0};
  double cruise_scale_{1.0};
  uint8_t mode_req_{1};              // V3 MODE_CAN
  bool strafe_jl_from_angular_{true};
  double strafe_speed_m_s_{0.3};
  int32_t sideways_steer_millirad_{1571};  // ≈ π/2 rad

  mutable std::mutex mutex_;

  // --- MCU 上行缓存 ---
  jetson_mcu_msgs::msg::V3Status latest_status_;
  jetson_mcu_msgs::msg::V3ExtStatus latest_ext_;
  bool has_status_{false};
  bool has_ext_{false};
  double last_status_time_{0.0};

  // --- 待下发运动（由 cmd_vel 写入，定时器读取）---
  int16_t pending_v_mm_s_{0};
  int16_t pending_omega_{0};
  int16_t pending_steer_{0};
  uint8_t pending_motion_model_{0};
  double last_cmd_time_{0.0};
  int16_t last_motion_v_mm_s_{0};
  int16_t last_motion_omega_{0};
  int16_t last_motion_steer_{0};
  double recover_until_{0.0};

  // --- 里程计积分状态 ---
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_theta_{0.0};
  std::optional<double> last_odom_time_;

  // --- MCU↔Jetson 时间同步 ---
  double offset_ms_{0.0};
  bool offset_valid_{false};

  // --- 灯光控制（MCU 要求 enable=1 才生效）---
  uint8_t light_enable_{1};
  uint8_t light_mode_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr light_sub_;
  rclcpp::Subscription<jetson_mcu_msgs::msg::V3Status>::SharedPtr status_sub_;
  rclcpp::Subscription<jetson_mcu_msgs::msg::V3ExtStatus>::SharedPtr ext_sub_;
  rclcpp::Subscription<jetson_mcu_msgs::msg::TimeSyncResponse>::SharedPtr time_sync_sub_;

  rclcpp::Publisher<jetson_mcu_msgs::msg::V3Command>::SharedPtr command_pub_;
  rclcpp::Publisher<scr_sensor::msg::VehicleData>::SharedPtr vehicle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace agv_base_driver
