#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/clock.hpp>

namespace imu_adapter
{

/// Jetson 单调时钟毫秒
[[nodiscard]] double monoMs();

/// 将采样时刻的 monotonic ms 映射为 ROS 时间戳
[[nodiscard]] builtin_interfaces::msg::Time stampFromMono(
  double sample_mono_ms,
  rclcpp::Clock & clock);

/// @brief MCU 虚拟时间轴辅助（与 BLOB/v3_status 对齐）
class McuVirtualTimeSync
{
public:
  void setUseTimeSync(bool use);
  void updateOffset(double offset_ms, bool offset_valid);

  /// @brief 为 IMU 采样时刻生成 header.stamp
  [[nodiscard]] builtin_interfaces::msg::Time stampAtMono(
    double sample_mono_ms,
    rclcpp::Clock & clock) const;

private:
  bool use_time_sync_{true};
  double offset_ms_{0.0};
  bool offset_valid_{false};
};

}  // namespace imu_adapter
