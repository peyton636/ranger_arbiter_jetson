#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/time.hpp>

#include "tracker/track.hpp"

namespace perception
{

/// 跟踪器可调参数
struct TrackManagerConfig
{
  double association_distance{0.15};   ///< 跨帧关联门控距离 (m)
  double position_alpha{0.35};         ///< EMA 系数，越小越平滑
  int min_hits{3};                     ///< 确认轨迹所需连续命中帧数
  int max_misses{5};                   ///< 允许连续丢失帧数，超出则删除轨迹
  double track_timeout_sec{0.5};       ///< 轨迹超时删除阈值 (s)
  int max_tracks{32};                  ///< 最大同时跟踪目标数
};

/// 跨帧目标跟踪管理器
///
/// 每帧处理流程：
/// 1. 贪心最近邻数据关联（门控距离 association_distance）
/// 2. 匹配轨迹 EMA 更新，未匹配轨迹 misses++
/// 3. 未匹配观测创建新轨迹
/// 4. 删除 misses 超限或超时的轨迹
/// 5. 仅输出 confirmed 轨迹的滤波位姿
class TrackManager
{
public:
  explicit TrackManager(TrackManagerConfig config);

  void setConfig(const TrackManagerConfig & config);

  /// 处理一帧观测，更新全部轨迹状态
  void update(const geometry_msgs::msg::PoseArray & measurements, const rclcpp::Time & now);

  [[nodiscard]] std::vector<Track> confirmedTracks() const;
  [[nodiscard]] std::vector<Track> allTracks() const;

  /// 将已确认轨迹组装为下游兼容的 PoseArray
  [[nodiscard]] geometry_msgs::msg::PoseArray buildOutputPoseArray(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const;

  void reset();

private:
  [[nodiscard]] static double distance(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);

  void pruneStaleTracks(const rclcpp::Time & now);

  TrackManagerConfig config_;
  std::vector<Track> tracks_;
  uint64_t next_id_{1};
};

}  // namespace perception
