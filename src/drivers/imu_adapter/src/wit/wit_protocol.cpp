#include "wit/wit_protocol.hpp"

#include <cmath>
#include <cstring>

#include "jetson/mcu_virtual_time_sync.hpp"

namespace imu_adapter
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kFrameSize = 11;

int16_t readInt16(const uint8_t * data)
{
  int16_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

}  // namespace

bool verifyChecksum(const uint8_t * frame, const size_t length)
{
  if (length < kFrameSize) {
    return false;
  }
  uint8_t sum = 0;
  for (size_t i = 0; i < 10; ++i) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return sum == frame[10];
}

bool feedFrame(const std::vector<uint8_t> & frame, WitImuState & state)
{
  if (frame.size() < kFrameSize || frame[0] != 0x55 || !verifyChecksum(frame.data(), frame.size())) {
    return false;
  }

  const int16_t raw0 = readInt16(&frame[2]);
  const int16_t raw1 = readInt16(&frame[4]);
  const int16_t raw2 = readInt16(&frame[6]);

  switch (frame[1]) {
    case static_cast<uint8_t>(WitFrameType::Acceleration):
      state.acceleration_m_s2 = {
        raw0 / 32768.0 * 16.0 * 9.8,
        raw1 / 32768.0 * 16.0 * 9.8,
        raw2 / 32768.0 * 16.0 * 9.8,
      };
      break;
    case static_cast<uint8_t>(WitFrameType::AngularVelocity):
      state.angular_velocity_rad_s = {
        raw0 / 32768.0 * 2000.0 * kPi / 180.0,
        raw1 / 32768.0 * 2000.0 * kPi / 180.0,
        raw2 / 32768.0 * 2000.0 * kPi / 180.0,
      };
      break;
    case static_cast<uint8_t>(WitFrameType::Angle):
      state.angle_deg = {
        raw0 / 32768.0 * 180.0,
        raw1 / 32768.0 * 180.0,
        raw2 / 32768.0 * 180.0,
      };
      state.sample_mono_ms = monoMs();
      return true;
    case static_cast<uint8_t>(WitFrameType::Magnetometer):
      state.magnetometer_raw = {raw0, raw1, raw2};
      break;
    default:
      break;
  }
  return false;
}

geometry_msgs::msg::Quaternion eulerToQuaternion(
  const double roll_rad, const double pitch_rad, const double yaw_rad)
{
  const double cy = std::cos(yaw_rad * 0.5);
  const double sy = std::sin(yaw_rad * 0.5);
  const double cp = std::cos(pitch_rad * 0.5);
  const double sp = std::sin(pitch_rad * 0.5);
  const double cr = std::cos(roll_rad * 0.5);
  const double sr = std::sin(roll_rad * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

void fillImuMessage(
  const WitImuState & state,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  sensor_msgs::msg::Imu & imu_out)
{
  const double roll = state.angle_deg[0] * kPi / 180.0;
  const double pitch = state.angle_deg[1] * kPi / 180.0;
  const double yaw = state.angle_deg[2] * kPi / 180.0;
  const auto q = eulerToQuaternion(roll, pitch, yaw);

  imu_out.header.stamp = stamp;
  imu_out.header.frame_id = frame_id;
  imu_out.orientation = q;
  imu_out.angular_velocity.x = state.angular_velocity_rad_s[0];
  imu_out.angular_velocity.y = state.angular_velocity_rad_s[1];
  imu_out.angular_velocity.z = state.angular_velocity_rad_s[2];
  imu_out.linear_acceleration.x = state.acceleration_m_s2[0];
  imu_out.linear_acceleration.y = state.acceleration_m_s2[1];
  imu_out.linear_acceleration.z = state.acceleration_m_s2[2];
}

void fillMagMessage(
  const WitImuState & state,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  sensor_msgs::msg::MagneticField & mag_out)
{
  mag_out.header.stamp = stamp;
  mag_out.header.frame_id = frame_id;
  mag_out.magnetic_field.x = static_cast<double>(state.magnetometer_raw[0]);
  mag_out.magnetic_field.y = static_cast<double>(state.magnetometer_raw[1]);
  mag_out.magnetic_field.z = static_cast<double>(state.magnetometer_raw[2]);
}

}  // namespace imu_adapter
