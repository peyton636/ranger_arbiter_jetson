#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace object_localization
{

class ObjectLocalizationNode : public rclcpp::Node
{
public:
  ObjectLocalizationNode();

private:
  using DetectionArray = vision_msgs::msg::Detection2DArray;
  using ImageMsg = sensor_msgs::msg::Image;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
  using PoseArrayMsg = geometry_msgs::msg::PoseArray;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    DetectionArray, ImageMsg, CameraInfoMsg>;

  void syncCallback(
    const DetectionArray::ConstSharedPtr & detections_msg,
    const ImageMsg::ConstSharedPtr & depth_msg,
    const CameraInfoMsg::ConstSharedPtr & camera_info_msg);

  void loadHandEyeYaml(const std::string & yaml_path);

  bool getDepthFromDetection(
    const cv::Mat & depth_image,
    const vision_msgs::msg::Detection2D & det,
    double & depth_m) const;

  bool transformPointToTarget(
    const tf2::Vector3 & p_camera,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & camera_frame,
    tf2::Vector3 & p_target) const;

  static tf2::Transform transformMsgToTf(const geometry_msgs::msg::Transform & msg);

private:
  rclcpp::Publisher<PoseArrayMsg>::SharedPtr pose_pub_;

  message_filters::Subscriber<DetectionArray> detections_sub_;
  message_filters::Subscriber<ImageMsg> depth_sub_;
  message_filters::Subscriber<CameraInfoMsg> camera_info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string detections_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string hand_eye_yaml_;

  std::string handeye_parent_frame_;
  std::string handeye_child_frame_;

  tf2::Transform handeye_parent_T_camera_;
  bool has_handeye_{false};

  int depth_window_{5};
  double depth_scale_{0.001};
  double min_depth_{0.1};
  double max_depth_{5.0};
  double bbox_roi_scale_{0.4};
};

}  // namespace object_localization