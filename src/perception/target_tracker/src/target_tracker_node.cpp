#include "target_tracker_node.hpp"

#include <sstream>

namespace perception
{

TargetTrackerNode::TargetTrackerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("target_tracker_node", options)
{
  declareParameters();
}

void TargetTrackerNode::declareParameters()
{
  // fusion_pose 输出的原始 3D 位姿数组
  input_topic_ = declare_parameter<std::string>(
    "input_topic", "/perception/object_pose_array");

  // 滤波/跟踪后的稳态位姿，供 task_fsm 等下游消费
  output_topic_ = declare_parameter<std::string>(
    "output_topic", "/perception/tracked_object_pose_array");

  // RViz 可视化话题，显示轨迹球体与 track id
  marker_topic_ = declare_parameter<std::string>(
    "marker_topic", "/perception/tracked_markers");
  publish_markers_ = declare_parameter<bool>("publish_markers", true);

  // 跟踪算法参数，详见 TrackManagerConfig
  track_config_.association_distance =
    declare_parameter<double>("association_distance", 0.15);
  track_config_.position_alpha =
    declare_parameter<double>("position_alpha", 0.35);
  track_config_.min_hits =
    declare_parameter<int>("min_hits", 3);
  track_config_.max_misses =
    declare_parameter<int>("max_misses", 5);
  track_config_.track_timeout_sec =
    declare_parameter<double>("track_timeout_sec", 0.5);
  track_config_.max_tracks =
    declare_parameter<int>("max_tracks", 32);
}

TrackManagerConfig TargetTrackerNode::loadTrackConfig()
{
  TrackManagerConfig cfg;
  cfg.association_distance = get_parameter("association_distance").as_double();
  cfg.position_alpha = get_parameter("position_alpha").as_double();
  cfg.min_hits = get_parameter("min_hits").as_int();
  cfg.max_misses = get_parameter("max_misses").as_int();
  cfg.track_timeout_sec = get_parameter("track_timeout_sec").as_double();
  cfg.max_tracks = get_parameter("max_tracks").as_int();
  return cfg;
}

TargetTrackerNode::CallbackReturn TargetTrackerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring TargetTrackerNode...");
  track_config_ = loadTrackConfig();
  track_manager_ = std::make_unique<TrackManager>(track_config_);

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);
  if (publish_markers_) {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  }

  RCLCPP_INFO(
    get_logger(), "Configured: input=%s output=%s",
    input_topic_.c_str(), output_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

TargetTrackerNode::CallbackReturn TargetTrackerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating TargetTrackerNode...");
  meas_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TargetTrackerNode::onMeasurements, this, std::placeholders::_1));

  pose_pub_->on_activate();
  if (marker_pub_) {
    marker_pub_->on_activate();
  }

  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

TargetTrackerNode::CallbackReturn TargetTrackerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating TargetTrackerNode...");
  meas_sub_.reset();
  pose_pub_->on_deactivate();
  if (marker_pub_) {
    marker_pub_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

TargetTrackerNode::CallbackReturn TargetTrackerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up TargetTrackerNode...");
  meas_sub_.reset();
  pose_pub_.reset();
  marker_pub_.reset();
  if (track_manager_) {
    track_manager_->reset();
  }
  track_manager_.reset();
  return CallbackReturn::SUCCESS;
}

TargetTrackerNode::CallbackReturn TargetTrackerNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

void TargetTrackerNode::onMeasurements(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (!track_manager_ || !pose_pub_ || !pose_pub_->is_activated()) {
    return;
  }

  const rclcpp::Time now(msg->header.stamp);
  track_manager_->update(*msg, now);

  const auto output = track_manager_->buildOutputPoseArray(
    msg->header.frame_id, msg->header.stamp);
  pose_pub_->publish(output);

  if (publish_markers_ && marker_pub_ && marker_pub_->is_activated()) {
    publishMarkers(
      track_manager_->confirmedTracks(),
      msg->header.frame_id,
      msg->header.stamp);
  }
}

void TargetTrackerNode::publishMarkers(
  const std::vector<Track> & tracks,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::MarkerArray array;

  // 先清除上一帧 marker，避免残留
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.ns = "target_tracker";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  int id = 1;
  for (const auto & track : tracks) {
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frame_id;
    sphere.header.stamp = stamp;
    sphere.ns = "target_tracker";
    sphere.id = id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = track.filteredPose();
    sphere.scale.x = 0.04;
    sphere.scale.y = 0.04;
    sphere.scale.z = 0.04;
    sphere.color.r = 0.1f;
    sphere.color.g = 0.8f;
    sphere.color.b = 0.2f;
    sphere.color.a = 0.9f;
    array.markers.push_back(sphere);

    visualization_msgs::msg::Marker text;
    text.header = sphere.header;
    text.ns = "target_tracker_id";
    text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = sphere.pose;
    text.pose.position.z += 0.06;
    text.scale.z = 0.04;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    text.text = "id:" + std::to_string(track.id);
    array.markers.push_back(text);
  }

  marker_pub_->publish(array);
}

}  // namespace perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception::TargetTrackerNode)
