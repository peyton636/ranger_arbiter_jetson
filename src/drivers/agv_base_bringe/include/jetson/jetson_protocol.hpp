#pragma once

#include <cstdint>

namespace agv_base_driver
{

/// cmd_vel 超过该时长未更新则触发零速恢复
constexpr int32_t kCmdTimeoutMs = 500;
/// 零速恢复窗口时长（ms），期间禁止任何运动指令
constexpr int32_t kRecoverStableMs = 1000;
/// V3 下行默认模式：CAN 指令控制
constexpr uint8_t kModeCan = 1;

/// V3 下行运动指令（单位与 jetson_mcu_msgs/V3Command 一致）
struct MotionCommand
{
  int16_t v_mm_s{0};              // 线速度 mm/s
  int16_t omega_millirad_s{0};    // 自旋角速度，0.001 rad/s
  int16_t steer_millirad{0};      // 转角，0.001 rad
  uint8_t motion_model{0};        // ACKermann / SIDEWAYS / SPIN
};

/// @brief 将 geometry_msgs/Twist 映射为 V3 运动模式
///
/// 优先级：纯自旋 > 斜移 > 阿克曼前后。
/// @param strafe_jl_from_angular true 时，仅 angular.z 输入映射为侧移（用于 JL 底盘遥控习惯）
/// @param sideways_steer_millirad 斜移模式下的固定转角（通常 ≈ 90°）
MotionCommand twistToMotion(
  double linear_x,
  double linear_y,
  double angular_z,
  bool strafe_jl_from_angular,
  double strafe_speed_m_s,
  int32_t sideways_steer_millirad);

}  // namespace agv_base_driver
