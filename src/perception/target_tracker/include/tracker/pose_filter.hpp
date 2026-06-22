#pragma once

#include <geometry_msgs/msg/point.hpp>

namespace perception
{

/// 单目标 3D 位置 EMA（指数滑动平均）滤波器
///
/// filtered = alpha * measurement + (1 - alpha) * filtered
/// alpha 越小输出越平滑，但响应越慢；抖动场景建议 0.2~0.35
class PoseFilter
{
public:
  explicit PoseFilter(double alpha = 0.35);

  void setAlpha(double alpha);
  void reset(const geometry_msgs::msg::Point & measurement);
  geometry_msgs::msg::Point update(const geometry_msgs::msg::Point & measurement);
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] const geometry_msgs::msg::Point & state() const { return state_; }

private:
  double alpha_;                        ///< EMA 系数，范围 [0.01, 1.0]
  geometry_msgs::msg::Point state_{};     ///< 当前滤波状态
  bool initialized_{false};
};

}  // namespace perception
