#include "object_localization_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <yaml-cpp/yaml.h>

namespace object_localization
{

ObjectLocalizationNode::ObjectLocalizationNode()
: Node("object_localization_node")
{
  detections_topic_ = this->declare_parameter<std::string>(
    "detections_topic", "/perception/detections_2d");
  depth_topic_ = this->declare_parameter<std::string>(
    "depth_topic", "/camera/depth/image_raw");
  camera_info_topic_ = this->declare_parameter<std::string>(
    "camera_info_topic", "/camera/camera_info");
  output_topic_ = this->declare_parameter<std::string>(
    "output_topic", "/perception/object_pose_array");

  target_frame_ = this->declare_parameter<std::string>("target_frame", "");
  hand_eye_yaml_ = this->declare_parameter<std::string>(
    "hand_eye_yaml",
    "/home/dingxiaoyi/workspace/cangyirobot/src/perception/object_localization/config/hand_eye_result.yaml");

  depth_window_ = this->declare_parameter<int>("depth_window", 5);
  depth_scale_ = this->declare_parameter<double>("depth_scale", 0.001);
  min_depth_ = this->declare_parameter<double>("min_depth", 0.1);
  max_depth_ = this->declare_parameter<double>("max_depth", 5.0);
  bbox_roi_scale_ = this->declare_parameter<double>("bbox_roi_scale", 0.4);

  if (depth_window_ <= 0) {
    depth_window_ = 5;
  }
  if (depth_window_ % 2 == 0) {
    depth_window_ += 1;
  }
  bbox_roi_scale_ = std::clamp(bbox_roi_scale_, 0.1, 1.0);

  pose_pub_ = this->create_publisher<PoseArrayMsg>(output_topic_, 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!hand_eye_yaml_.empty()) {
    loadHandEyeYaml(hand_eye_yaml_);
  }

  detections_sub_.subscribe(this, detections_topic_, rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
  camera_info_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(20), detections_sub_, depth_sub_, camera_info_sub_);
  sync_->registerCallback(
    std::bind(
      &ObjectLocalizationNode::syncCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

  RCLCPP_INFO(this->get_logger(), "object_localization_node started");
  RCLCPP_INFO(this->get_logger(), "detections_topic: %s", detections_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "depth_topic: %s", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "camera_info_topic: %s", camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "target_frame: %s",
    target_frame_.empty() ? "<camera_frame>" : target_frame_.c_str());
}

void ObjectLocalizationNode::loadHandEyeYaml(const std::string & yaml_path)
{
  YAML::Node root = YAML::LoadFile(yaml_path);

  handeye_parent_frame_ = root["parent_frame"].as<std::string>();
  handeye_child_frame_ = root["child_frame"].as<std::string>();

  const auto t = root["translation"];
  const auto q = root["rotation_quaternion"];

  const double tx = t["x"].as<double>();
  const double ty = t["y"].as<double>();
  const double tz = t["z"].as<double>();

  const double qx = q["x"].as<double>();
  const double qy = q["y"].as<double>();
  const double qz = q["z"].as<double>();
  const double qw = q["w"].as<double>();

  tf2::Quaternion quat(qx, qy, qz, qw);
  quat.normalize();

  handeye_parent_T_camera_ = tf2::Transform(quat, tf2::Vector3(tx, ty, tz));
  has_handeye_ = true;

  RCLCPP_INFO(this->get_logger(), "Loaded hand-eye yaml: %s", yaml_path.c_str());
  RCLCPP_INFO(
    this->get_logger(), "hand-eye transform: %s <- %s",
    handeye_parent_frame_.c_str(), handeye_child_frame_.c_str());
}

tf2::Transform ObjectLocalizationNode::transformMsgToTf(const geometry_msgs::msg::Transform & msg)
{
  tf2::Quaternion q(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
  q.normalize();
  tf2::Vector3 t(msg.translation.x, msg.translation.y, msg.translation.z);
  return tf2::Transform(q, t);
}

bool ObjectLocalizationNode::getDepthFromDetection(
  const cv::Mat & depth_image,
  const vision_msgs::msg::Detection2D & det,
  double & depth_m) const
{
  if (depth_image.empty()) {
    return false;
  }

  const double center_u = det.bbox.center.x;
  const double center_v = det.bbox.center.y;
  const double box_w = det.bbox.size_x;
  const double box_h = det.bbox.size_y;

  if (!std::isfinite(center_u) || !std::isfinite(center_v)) {
    return false;
  }

  std::vector<double> valid_depths;
  valid_depths.reserve(256);

  auto push_depth = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= depth_image.cols || y >= depth_image.rows) {
      return;
    }

    double d = 0.0;

    if (depth_image.type() == CV_16UC1) {
      const uint16_t raw = depth_image.at<uint16_t>(y, x);
      if (raw == 0U) {
        return;
      }
      d = static_cast<double>(raw) * depth_scale_;
    } else if (depth_image.type() == CV_32FC1) {
      const float raw = depth_image.at<float>(y, x);
      if (!std::isfinite(raw) || raw <= 0.0f) {
        return;
      }
      d = static_cast<double>(raw);
    } else {
      return;
    }

    if (std::isfinite(d) && d >= min_depth_ && d <= max_depth_) {
      valid_depths.push_back(d);
    }
  };

  if (box_w > 1.0 && box_h > 1.0) {
    const int roi_w = std::max(1, static_cast<int>(std::round(box_w * bbox_roi_scale_)));
    const int roi_h = std::max(1, static_cast<int>(std::round(box_h * bbox_roi_scale_)));
    const int x0 = std::max(0, static_cast<int>(std::round(center_u)) - roi_w / 2);
    const int y0 = std::max(0, static_cast<int>(std::round(center_v)) - roi_h / 2);
    const int x1 = std::min(depth_image.cols - 1, x0 + roi_w - 1);
    const int y1 = std::min(depth_image.rows - 1, y0 + roi_h - 1);

    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        push_depth(x, y);
      }
    }
  }

  if (valid_depths.empty()) {
    const int u = static_cast<int>(std::round(center_u));
    const int v = static_cast<int>(std::round(center_v));
    const int half = depth_window_ / 2;

    for (int y = std::max(0, v - half); y <= std::min(depth_image.rows - 1, v + half); ++y) {
      for (int x = std::max(0, u - half); x <= std::min(depth_image.cols - 1, u + half); ++x) {
        push_depth(x, y);
      }
    }
  }

  if (valid_depths.empty()) {
    return false;
  }

  auto mid = valid_depths.begin() + valid_depths.size() / 2;
  std::nth_element(valid_depths.begin(), mid, valid_depths.end());
  depth_m = *mid;
  return true;
}

bool ObjectLocalizationNode::transformPointToTarget(
  const tf2::Vector3 & p_camera,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & camera_frame,
  tf2::Vector3 & p_target) const
{
  if (target_frame_.empty() || target_frame_ == camera_frame) {
    p_target = p_camera;
    return true;
  }

  if (has_handeye_ && camera_frame == handeye_child_frame_) {
    const tf2::Vector3 p_parent = handeye_parent_T_camera_ * p_camera;

    if (target_frame_ == handeye_parent_frame_) {
      p_target = p_parent;
      return true;
    }

    try {
      const auto tf_target_parent = tf_buffer_->lookupTransform(
        target_frame_,
        handeye_parent_frame_,
        rclcpp::Time(stamp),
        rclcpp::Duration::from_seconds(0.1));

      const tf2::Transform target_T_parent = transformMsgToTf(tf_target_parent.transform);
      p_target = target_T_parent * p_parent;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "lookupTransform(%s <- %s) failed: %s",
        target_frame_.c_str(), handeye_parent_frame_.c_str(), e.what());
      return false;
    }
  }

  try {
    const auto tf_target_camera = tf_buffer_->lookupTransform(
      target_frame_,
      camera_frame,
      rclcpp::Time(stamp),
      rclcpp::Duration::from_seconds(0.1));

    const tf2::Transform target_T_camera = transformMsgToTf(tf_target_camera.transform);
    p_target = target_T_camera * p_camera;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      this->get_logger(),
      "lookupTransform(%s <- %s) failed: %s",
      target_frame_.c_str(), camera_frame.c_str(), e.what());
    return false;
  }
}

void ObjectLocalizationNode::syncCallback(
  const DetectionArray::ConstSharedPtr & detections_msg,
  const ImageMsg::ConstSharedPtr & depth_msg,
  const CameraInfoMsg::ConstSharedPtr & camera_info_msg)
{
  const double fx = camera_info_msg->k[0];
  const double fy = camera_info_msg->k[4];
  const double cx = camera_info_msg->k[2];
  const double cy = camera_info_msg->k[5];

  if (fx <= 0.0 || fy <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid camera intrinsics");
    return;
  }

  cv_bridge::CvImageConstPtr depth_cv_ptr;
  try {
    depth_cv_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat & depth_image = depth_cv_ptr->image;
  if (depth_image.empty()) {
    return;
  }

  std::string camera_frame = camera_info_msg->header.frame_id;
  if (camera_frame.empty()) {
    camera_frame = depth_msg->header.frame_id;
  }
  if (camera_frame.empty()) {
    camera_frame = handeye_child_frame_;
  }

  PoseArrayMsg pose_array;
  pose_array.header.stamp = detections_msg->header.stamp;
  pose_array.header.frame_id = target_frame_.empty() ? camera_frame : target_frame_;

  for (const auto & det : detections_msg->detections) {
    double depth_m = 0.0;
    if (!getDepthFromDetection(depth_image, det, depth_m)) {
      continue;
    }

    const double u = det.bbox.center.x;
    const double v = det.bbox.center.y;

    const double x = (u - cx) * depth_m / fx;
    const double y = (v - cy) * depth_m / fy;
    const double z = depth_m;

    tf2::Vector3 p_camera(x, y, z);
    tf2::Vector3 p_target;
    if (!transformPointToTarget(p_camera, detections_msg->header.stamp, camera_frame, p_target)) {
      continue;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = p_target.x();
    pose.position.y = p_target.y();
    pose.position.z = p_target.z();
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    pose_array.poses.push_back(pose);
  }

  pose_pub_->publish(pose_array);
}

}  // namespace object_localization