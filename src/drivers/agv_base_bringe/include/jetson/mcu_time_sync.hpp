#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/clock.hpp>

namespace agv_base_driver
{

/// @brief 将 MCU 虚拟时间轴映射为 ROS 时间戳
///
/// 网关 time_sync 提供 offset_ms，满足 mcu_tick ≈ jetson_mono + offset。
/// 下行 V3Command 使用该时间戳，以便与 v3_status/BLOB 处于同一事件时间轴。
/// @param fallback offset 无效时使用的备用 stamp（通常为上行帧 header.stamp）
builtin_interfaces::msg::Time mcuVirtualToRosStamp(
  double offset_ms,
  bool offset_valid,
  rclcpp::Clock & clock,
  const builtin_interfaces::msg::Time * fallback = nullptr);

}  // namespace agv_base_driver
