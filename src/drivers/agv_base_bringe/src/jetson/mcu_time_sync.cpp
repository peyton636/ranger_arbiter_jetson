#include "jetson/mcu_time_sync.hpp"

#include <cmath>

namespace agv_base_driver
{

namespace
{

builtin_interfaces::msg::Time toTimeMsg(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time stamp;
  const int64_t ns = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(ns / 1'000'000'000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
  return stamp;
}

}  // namespace

builtin_interfaces::msg::Time mcuVirtualToRosStamp(
  const double offset_ms,
  const bool offset_valid,
  rclcpp::Clock & clock,
  const builtin_interfaces::msg::Time * fallback)
{
  if (!offset_valid) {
    if (fallback != nullptr) {
      return *fallback;
    }
    // 尚未完成 time_sync 时退化为 ROS 当前时间
    return toTimeMsg(clock.now());
  }

  // mcu_virtual ≈ jetson_now + offset_ms
  const auto now = clock.now();
  const int64_t offset_ns = static_cast<int64_t>(std::round(offset_ms * 1'000'000.0));
  return toTimeMsg(now + rclcpp::Duration::from_nanoseconds(offset_ns));
}

}  // namespace agv_base_driver
