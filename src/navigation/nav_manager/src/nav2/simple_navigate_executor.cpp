#include "nav2/simple_navigate_executor.hpp"

#include <algorithm>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace navigation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

SimpleNavigateExecutor::SimpleNavigateExecutor(SimpleNavigateConfig config)
: config_(std::move(config))
{
}

void SimpleNavigateExecutor::setConfig(const SimpleNavigateConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

void SimpleNavigateExecutor::setResultCallback(
  std::function<void(NavActionStatus, const std::string &)> cb)
{
  std::lock_guard<std::mutex> lock(mutex_);
  result_cb_ = std::move(cb);
}

NavActionStatus SimpleNavigateExecutor::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool SimpleNavigateExecutor::isBusy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_ == NavActionStatus::PENDING;
}

bool SimpleNavigateExecutor::hasGoal() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_goal_;
}

bool SimpleNavigateExecutor::sendGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == NavActionStatus::PENDING) {
    return false;
  }
  goal_ = goal;
  has_goal_ = true;
  status_ = NavActionStatus::PENDING;
  return true;
}

void SimpleNavigateExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_goal_ = false;
  status_ = NavActionStatus::CANCELED;
}

void SimpleNavigateExecutor::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_goal_ = false;
  status_ = NavActionStatus::IDLE;
}

double SimpleNavigateExecutor::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

double SimpleNavigateExecutor::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

std::optional<geometry_msgs::msg::Twist> SimpleNavigateExecutor::update(
  const nav_msgs::msg::Odometry & odom)
{
  std::function<void(NavActionStatus, const std::string &)> cb;
  geometry_msgs::msg::PoseStamped goal;
  bool active = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_goal_ || status_ != NavActionStatus::PENDING) {
      return std::nullopt;
    }
    goal = goal_;
    active = true;
  }

  if (!active) {
    return std::nullopt;
  }

  const double x = odom.pose.pose.position.x;
  const double y = odom.pose.pose.position.y;
  const double yaw = yawFromQuaternion(odom.pose.pose.orientation);

  const double dx = goal.pose.position.x - x;
  const double dy = goal.pose.position.y - y;
  const double dist = std::hypot(dx, dy);
  const double target_yaw = std::atan2(dy, dx);
  const double yaw_err = normalizeAngle(target_yaw - yaw);

  geometry_msgs::msg::Twist cmd;

  if (dist <= config_.goal_tolerance_xy) {
    std::lock_guard<std::mutex> lock(mutex_);
    has_goal_ = false;
    status_ = NavActionStatus::SUCCEEDED;
    cb = result_cb_;
  } else {
    cmd.linear.x = std::clamp(config_.linear_gain * dist, 0.0, config_.max_linear_vel);
    cmd.angular.z = std::clamp(config_.angular_gain * yaw_err, -config_.max_angular_vel, config_.max_angular_vel);
  }

  if (cb) {
    cb(NavActionStatus::SUCCEEDED, "simple navigation succeeded");
    return geometry_msgs::msg::Twist{};
  }

  return cmd;
}

}  // namespace navigation
