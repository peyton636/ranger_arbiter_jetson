#include "tracker/pose_filter.hpp"

#include <algorithm>

namespace perception
{

PoseFilter::PoseFilter(double alpha)
: alpha_(std::clamp(alpha, 0.01, 1.0))
{
}

void PoseFilter::setAlpha(double alpha)
{
  alpha_ = std::clamp(alpha, 0.01, 1.0);
}

void PoseFilter::reset(const geometry_msgs::msg::Point & measurement)
{
  state_ = measurement;
  initialized_ = true;
}

geometry_msgs::msg::Point PoseFilter::update(const geometry_msgs::msg::Point & measurement)
{
  if (!initialized_) {
    reset(measurement);
    return state_;
  }

  // EMA：新观测权重 alpha，历史状态权重 (1 - alpha)
  const double beta = 1.0 - alpha_;
  state_.x = alpha_ * measurement.x + beta * state_.x;
  state_.y = alpha_ * measurement.y + beta * state_.y;
  state_.z = alpha_ * measurement.z + beta * state_.z;
  return state_;
}

}  // namespace perception
