#pragma once

#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "types/nav_types.hpp"

namespace navigation
{

struct SimpleNavigateConfig
{
  double goal_tolerance_xy{0.10};
  double goal_tolerance_yaw{0.15};
  double max_linear_vel{0.3};
  double max_angular_vel{0.8};
  double linear_gain{0.8};
  double angular_gain{1.5};
};

/// 简易 go-to-goal 执行器（无 Nav2 时使用，也可作为 Nav2 不可用时的回退）
class SimpleNavigateExecutor
{
public:
  explicit SimpleNavigateExecutor(SimpleNavigateConfig config);

  void setConfig(const SimpleNavigateConfig & config);
  void setResultCallback(std::function<void(NavActionStatus, const std::string &)> cb);

  [[nodiscard]] NavActionStatus status() const;
  [[nodiscard]] bool isBusy() const;
  [[nodiscard]] bool hasGoal() const;

  bool sendGoal(const geometry_msgs::msg::PoseStamped & goal);
  void cancel();
  void reset();

  /// 根据当前里程计计算速度指令；到达目标时触发 result 回调
  std::optional<geometry_msgs::msg::Twist> update(
    const nav_msgs::msg::Odometry & odom);

private:
  [[nodiscard]] static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  [[nodiscard]] static double normalizeAngle(double angle);

  SimpleNavigateConfig config_;
  std::function<void(NavActionStatus, const std::string &)> result_cb_;

  mutable std::mutex mutex_;
  NavActionStatus status_{NavActionStatus::IDLE};
  geometry_msgs::msg::PoseStamped goal_;
  bool has_goal_{false};
};

}  // namespace navigation
