#include "jetson/mcu_virtual_time_sync.hpp"

#include <chrono>
#include <cmath>

namespace imu_adapter
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

double monoMs()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double, std::milli>(now).count();
}

void McuVirtualTimeSync::setUseTimeSync(const bool use)
{
  use_time_sync_ = use;
}

void McuVirtualTimeSync::updateOffset(const double offset_ms, const bool offset_valid)
{
  offset_ms_ = offset_ms;
  offset_valid_ = offset_valid;
}

builtin_interfaces::msg::Time stampFromMono(
  const double sample_mono_ms,
  rclcpp::Clock & clock)
{
  const int64_t delta_ns = static_cast<int64_t>(
    std::round((sample_mono_ms - monoMs()) * 1'000'000.0));
  return toTimeMsg(clock.now() + rclcpp::Duration::from_nanoseconds(delta_ns));
}

builtin_interfaces::msg::Time McuVirtualTimeSync::stampAtMono(
  const double sample_mono_ms,
  rclcpp::Clock & clock) const
{
  if (!use_time_sync_) {
    return toTimeMsg(clock.now());
  }

  const double mono = sample_mono_ms > 0.0 ? sample_mono_ms : monoMs();
  return stampFromMono(mono, clock);
}

}  // namespace imu_adapter
