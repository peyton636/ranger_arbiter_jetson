#include "tracker/track_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace perception
{

namespace
{

geometry_msgs::msg::Point poseToPoint(const geometry_msgs::msg::Pose & pose)
{
  return pose.position;
}

}  // namespace

void Track::predictMiss()
{
  ++misses;
}

void Track::updateMeasurement(
  const geometry_msgs::msg::Point & measurement,
  const builtin_interfaces::msg::Time & stamp,
  int min_hits)
{
  last_measurement = measurement;
  last_seen = stamp;
  filter.update(measurement);
  ++hits;
  misses = 0;
  if (!confirmed && hits >= min_hits) {
    confirmed = true;
  }
}

geometry_msgs::msg::Pose Track::filteredPose() const
{
  geometry_msgs::msg::Pose pose;
  pose.position = filter.state();
  pose.orientation.w = 1.0;
  return pose;
}

TrackManager::TrackManager(TrackManagerConfig config)
: config_(std::move(config))
{
}

void TrackManager::setConfig(const TrackManagerConfig & config)
{
  config_ = config;
  for (auto & track : tracks_) {
    track.filter.setAlpha(config_.position_alpha);
  }
}

double TrackManager::distance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void TrackManager::update(
  const geometry_msgs::msg::PoseArray & measurements,
  const rclcpp::Time & now)
{
  std::vector<bool> track_matched(tracks_.size(), false);
  std::vector<bool> meas_matched(measurements.poses.size(), false);

  struct Candidate
  {
    size_t track_idx;
    size_t meas_idx;
    double dist;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(tracks_.size() * measurements.poses.size());

  // 1. 构建所有满足门控距离的 (轨迹, 观测) 候选对
  for (size_t ti = 0; ti < tracks_.size(); ++ti) {
    for (size_t mi = 0; mi < measurements.poses.size(); ++mi) {
      const double dist = distance(
        tracks_[ti].filter.state(),
        poseToPoint(measurements.poses[mi]));
      if (dist <= config_.association_distance) {
        candidates.push_back({ti, mi, dist});
      }
    }
  }

  // 2. 按距离升序贪心匹配，保证每个轨迹/观测最多匹配一次
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      return a.dist < b.dist;
    });

  for (const auto & c : candidates) {
    if (track_matched[c.track_idx] || meas_matched[c.meas_idx]) {
      continue;
    }
    track_matched[c.track_idx] = true;
    meas_matched[c.meas_idx] = true;

    tracks_[c.track_idx].updateMeasurement(
      poseToPoint(measurements.poses[c.meas_idx]),
      measurements.header.stamp,
      config_.min_hits);
  }

  // 3. 未匹配的已有轨迹：递增 misses
  for (size_t ti = 0; ti < tracks_.size(); ++ti) {
    if (!track_matched[ti]) {
      tracks_[ti].predictMiss();
    }
  }

  // 4. 未匹配的观测：创建新轨迹
  for (size_t mi = 0; mi < measurements.poses.size(); ++mi) {
    if (meas_matched[mi]) {
      continue;
    }
    if (static_cast<int>(tracks_.size()) >= config_.max_tracks) {
      break;
    }

    Track track;
    track.id = next_id_++;
    track.filter.setAlpha(config_.position_alpha);
    track.filter.reset(poseToPoint(measurements.poses[mi]));
    track.updateMeasurement(
      poseToPoint(measurements.poses[mi]),
      measurements.header.stamp,
      config_.min_hits);
    tracks_.push_back(std::move(track));
  }

  // 5. 删除连续丢失超限的轨迹
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & track) {
        return track.misses > config_.max_misses;
      }),
    tracks_.end());

  pruneStaleTracks(now);
}

void TrackManager::pruneStaleTracks(const rclcpp::Time & now)
{
  const rclcpp::Time timeout = now - rclcpp::Duration::from_seconds(config_.track_timeout_sec);

  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&timeout](const Track & track) {
        const rclcpp::Time last_seen(track.last_seen);
        return last_seen < timeout;
      }),
    tracks_.end());
}

std::vector<Track> TrackManager::confirmedTracks() const
{
  std::vector<Track> out;
  out.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    if (track.confirmed) {
      out.push_back(track);
    }
  }
  return out;
}

std::vector<Track> TrackManager::allTracks() const
{
  return tracks_;
}

geometry_msgs::msg::PoseArray TrackManager::buildOutputPoseArray(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp) const
{
  geometry_msgs::msg::PoseArray out;
  out.header.stamp = stamp;
  out.header.frame_id = frame_id;

  // 仅输出已确认轨迹，抑制单帧噪声和短暂误检
  for (const auto & track : tracks_) {
    if (!track.confirmed) {
      continue;
    }
    out.poses.push_back(track.filteredPose());
  }
  return out;
}

void TrackManager::reset()
{
  tracks_.clear();
  next_id_ = 1;
}

}  // namespace perception
