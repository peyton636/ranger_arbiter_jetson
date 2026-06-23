#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace imu_adapter
{

/// WIT 协议帧类型
enum class WitFrameType : uint8_t
{
  Acceleration = 0x51,
  AngularVelocity = 0x52,
  Angle = 0x53,
  Magnetometer = 0x54,
};

/// 累积一帧 IMU 所需的 WIT 传感器状态
struct WitImuState
{
  std::array<double, 3> acceleration_m_s2{{0.0, 0.0, 0.0}};
  std::array<double, 3> angular_velocity_rad_s{{0.0, 0.0, 0.0}};
  std::array<double, 3> angle_deg{{0.0, 0.0, 0.0}};
  std::array<int16_t, 3> magnetometer_raw{{0, 0, 0}};
  std::optional<double> sample_mono_ms;
};

/// @brief 校验 WIT 11 字节帧
[[nodiscard]] bool verifyChecksum(const uint8_t * frame, size_t length);

/// @brief 解析单帧并更新状态；角度帧 (0x53) 返回 true 表示可发布
[[nodiscard]] bool feedFrame(const std::vector<uint8_t> & frame, WitImuState & state);

/// 欧拉角 (rad) → 四元数
[[nodiscard]] geometry_msgs::msg::Quaternion eulerToQuaternion(
  double roll_rad, double pitch_rad, double yaw_rad);

void fillImuMessage(
  const WitImuState & state,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  sensor_msgs::msg::Imu & imu_out);

void fillMagMessage(
  const WitImuState & state,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  sensor_msgs::msg::MagneticField & mag_out);

}  // namespace imu_adapter
