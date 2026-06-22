#pragma once

#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "tracker/pose_filter.hpp"

namespace perception
{

/// 单条跟踪轨迹的状态
struct Track
{
  uint64_t id{0};                                   ///< 全局唯一轨迹 ID
  PoseFilter filter;                                ///< 位置 EMA 滤波器
  geometry_msgs::msg::Point last_measurement{};     ///< 最近一次原始观测
  builtin_interfaces::msg::Time last_seen{};        ///< 最近一次更新时间戳
  int hits{0};                                      ///< 累计命中帧数
  int misses{0};                                    ///< 连续丢失帧数
  bool confirmed{false};                            ///< 是否已通过 min_hits 确认

  /// 本帧未匹配到观测，递增 misses
  void predictMiss();

  /// 匹配到新观测：更新滤波状态并重置 misses
  void updateMeasurement(
    const geometry_msgs::msg::Point & measurement,
    const builtin_interfaces::msg::Time & stamp,
    int min_hits);

  /// 返回滤波后的 Pose（姿态固定为单位四元数）
  [[nodiscard]] geometry_msgs::msg::Pose filteredPose() const;
};

}  // namespace perception
