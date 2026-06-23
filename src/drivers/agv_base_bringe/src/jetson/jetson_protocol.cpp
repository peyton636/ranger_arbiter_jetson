#include "jetson/jetson_protocol.hpp"

#include <algorithm>
#include <cmath>

#include <jetson_mcu_msgs/msg/v3_command.hpp>

namespace agv_base_driver
{

namespace
{

constexpr double kEps = 1e-6;

int16_t clampI16(int32_t value)
{
  return static_cast<int16_t>(std::clamp(value, static_cast<int32_t>(-32768), static_cast<int32_t>(32767)));
}

}  // namespace

MotionCommand twistToMotion(
  const double linear_x,
  const double linear_y,
  const double angular_z,
  const bool strafe_jl_from_angular,
  const double strafe_speed_m_s,
  const int32_t sideways_steer_millirad)
{
  MotionCommand out;

  const bool has_lx = std::abs(linear_x) > kEps;
  const bool has_ly = std::abs(linear_y) > kEps;
  const bool has_az = std::abs(angular_z) > kEps;

  // 1) 纯自旋：仅 angular.z，且无前后/侧向分量
  if (has_az && !has_lx && !has_ly && !strafe_jl_from_angular) {
    out.motion_model = jetson_mcu_msgs::msg::V3Command::MOTION_SPIN;
    out.omega_millirad_s = clampI16(static_cast<int32_t>(std::lround(angular_z * 1000.0)));
    return out;
  }

  // 2) 斜移：linear.y 或（JL 模式下）angular.z 映射为侧移
  if (has_ly || (strafe_jl_from_angular && has_az && !has_lx)) {
    out.motion_model = jetson_mcu_msgs::msg::V3Command::MOTION_SIDEWAYS;
    double speed_m_s = linear_y;
    int steer_sign = 1;

    if (strafe_jl_from_angular && !has_ly && has_az) {
      // JL 遥控习惯：左右拨杆 → 固定侧移速度 + 固定转角
      steer_sign = angular_z > 0.0 ? 1 : -1;
      speed_m_s = steer_sign * strafe_speed_m_s;
    } else if (has_ly) {
      steer_sign = linear_y > 0.0 ? 1 : -1;
      speed_m_s = linear_y;
    }

    out.v_mm_s = clampI16(static_cast<int32_t>(std::lround(speed_m_s * 1000.0)));
    out.steer_millirad = clampI16(steer_sign * sideways_steer_millirad);
    return out;
  }

  // 3) 阿克曼：前后行驶，可同时带自旋分量
  out.motion_model = jetson_mcu_msgs::msg::V3Command::MOTION_ACKERMANN;
  out.v_mm_s = clampI16(static_cast<int32_t>(std::lround(linear_x * 1000.0)));
  if (has_az) {
    out.omega_millirad_s = clampI16(static_cast<int32_t>(std::lround(angular_z * 1000.0)));
  }
  return out;
}

}  // namespace agv_base_driver
