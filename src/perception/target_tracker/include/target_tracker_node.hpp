#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tracker/track_manager.hpp"
#include "visibility_control.hpp"

namespace perception
{

/// @brief 目标跟踪 Lifecycle 节点
///
/// 职责：只负责"看见并理解目标"，不做任务编排。
/// - 订阅 fusion_pose 发布的 /perception/object_pose_array
/// - 跨帧数据关联 + EMA 时序滤波，输出稳态位姿
/// - 可选发布 MarkerArray 用于 RViz 调试（含 track id）
///
/// 数据流：
///   fusion_pose → object_pose_array → target_tracker → tracked_object_pose_array → task_fsm
class TARGET_TRACKER_PUBLIC TargetTrackerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TargetTrackerNode(const rclcpp::NodeOptions & options);
  ~TargetTrackerNode() override = default;

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declareParameters();
  TrackManagerConfig loadTrackConfig();
  void onMeasurements(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void publishMarkers(
    const std::vector<Track> & tracks,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp);

  TrackManagerConfig track_config_;
  std::unique_ptr<TrackManager> track_manager_;

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  bool publish_markers_{true};

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr meas_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace perception
