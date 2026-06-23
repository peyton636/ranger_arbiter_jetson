#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <jetson_mcu_msgs/msg/time_sync_response.hpp>

#include "jetson/mcu_virtual_time_sync.hpp"
#include "serial/serial_port.hpp"
#include "visibility_control.hpp"
#include "wit/wit_protocol.hpp"

namespace imu_adapter
{

/// @brief WIT IMU 串口适配 Lifecycle 节点
///
/// 职责：读取 WIT 11 字节协议帧，发布 Imu + MagneticField。
/// - 角度帧 (0x53) 到达时触发发布（与旧 wit_imu 行为一致）
/// - 可选订阅 /jetson_{link_type}/time_sync，将 stamp 对齐 MCU 虚拟时间轴
class IMU_ADAPTER_PUBLIC ImuAdapterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ImuAdapterNode(const rclcpp::NodeOptions & options);
  ~ImuAdapterNode() override;

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declareParameters();
  void onTimeSync(const jetson_mcu_msgs::msg::TimeSyncResponse::SharedPtr msg);
  void pollSerial();

  void publishImu(const WitImuState & state);

  std::string port_;
  int baud_{9600};
  std::string frame_id_;
  std::string imu_topic_;
  std::string mag_topic_;
  std::string link_type_;
  std::string time_sync_topic_;
  bool use_time_sync_stamp_{true};
  double poll_period_s_{0.01};

  SerialPort serial_;
  std::vector<uint8_t> rx_buffer_;
  WitImuState wit_state_;
  McuVirtualTimeSync time_sync_;

  rclcpp::Subscription<jetson_mcu_msgs::msg::TimeSyncResponse>::SharedPtr time_sync_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace imu_adapter
