#pragma once

#include <limits>
#include <mutex>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace navigation
{

/// 激光雷达安全监控：检测前方扇区障碍，对 cmd_vel 做门控
struct LidarSafetyConfig
{
  double stop_distance{0.35};       ///< 紧急停车距离 (m)
  double slow_distance{0.60};       ///< 开始减速距离 (m)
  double slow_scale{0.4};           ///< 减速区速度缩放系数
  double front_angle_deg{60.0};     ///< 前方监控扇区半角 (deg)
  double min_valid_range{0.10};     ///< 有效量程下限 (m)
};

class LidarSafetyMonitor
{
public:
  explicit LidarSafetyMonitor(LidarSafetyConfig config);

  void setConfig(const LidarSafetyConfig & config);
  void updateScan(const sensor_msgs::msg::LaserScan & scan);

  [[nodiscard]] bool hasScan() const;
  [[nodiscard]] bool isBlocked() const;
  [[nodiscard]] double nearestObstacleM() const;

  /// 对 Nav2 输出的速度做安全门控（blocked 时输出零速）
  geometry_msgs::msg::Twist filterCmdVel(const geometry_msgs::msg::Twist & cmd) const;

private:
  [[nodiscard]] std::optional<double> minRangeInFrontArc(
    const sensor_msgs::msg::LaserScan & scan) const;

  LidarSafetyConfig config_;
  mutable std::mutex mutex_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool has_scan_{false};
  bool blocked_{false};
  double nearest_obstacle_m_{std::numeric_limits<double>::infinity()};
};

}  // namespace navigation
