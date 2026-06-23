#include "safety/lidar_safety_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double degToRad(double deg)
{
  return deg * kPi / 180.0;
}

}  // namespace

LidarSafetyMonitor::LidarSafetyMonitor(LidarSafetyConfig config)
: config_(std::move(config))
{
}

void LidarSafetyMonitor::setConfig(const LidarSafetyConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

void LidarSafetyMonitor::updateScan(const sensor_msgs::msg::LaserScan & scan)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_scan_ = scan;
  has_scan_ = true;

  const auto min_range = minRangeInFrontArc(scan);
  if (!min_range.has_value()) {
    blocked_ = false;
    nearest_obstacle_m_ = std::numeric_limits<double>::infinity();
    return;
  }

  nearest_obstacle_m_ = min_range.value();
  blocked_ = nearest_obstacle_m_ <= config_.stop_distance;
}

bool LidarSafetyMonitor::hasScan() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_scan_;
}

bool LidarSafetyMonitor::isBlocked() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return blocked_;
}

double LidarSafetyMonitor::nearestObstacleM() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return nearest_obstacle_m_;
}

geometry_msgs::msg::Twist LidarSafetyMonitor::filterCmdVel(
  const geometry_msgs::msg::Twist & cmd) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  geometry_msgs::msg::Twist out = cmd;
  if (!has_scan_) {
    return out;
  }

  if (blocked_) {
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.angular.z = 0.0;
    return out;
  }

  if (nearest_obstacle_m_ <= config_.slow_distance) {
    const double ratio = std::clamp(
      (nearest_obstacle_m_ - config_.stop_distance) /
      std::max(config_.slow_distance - config_.stop_distance, 1e-3),
      config_.slow_scale, 1.0);
    out.linear.x *= ratio;
    out.linear.y *= ratio;
  }

  return out;
}

std::optional<double> LidarSafetyMonitor::minRangeInFrontArc(
  const sensor_msgs::msg::LaserScan & scan) const
{
  if (scan.ranges.empty()) {
    return std::nullopt;
  }

  const double half_arc = degToRad(config_.front_angle_deg);
  double min_range = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (std::abs(angle) > half_arc) {
      continue;
    }

    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < config_.min_valid_range) {
      continue;
    }
    if (r < scan.range_min || r > scan.range_max) {
      continue;
    }

    min_range = std::min(min_range, static_cast<double>(r));
  }

  if (!std::isfinite(min_range)) {
    return std::nullopt;
  }
  return min_range;
}

}  // namespace navigation
