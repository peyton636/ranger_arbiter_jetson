#include "tf_manager/static_tf_manager.hpp"
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <stdexcept>

namespace tf_manager
{

StaticTfManager::StaticTfManager(rclcpp::Node::SharedPtr node)
: node_(node)
{
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  RCLCPP_INFO(node_->get_logger(), "[StaticTfManager] Initialized.");
}

int StaticTfManager::loadAndPublish(const std::string & yaml_path)
{
  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "[StaticTfManager] Failed to load YAML: %s -> %s", yaml_path.c_str(), e.what());
    return 0;
  }

  if (!config["static_transforms"]) {
    RCLCPP_WARN(node_->get_logger(), "[StaticTfManager] No 'static_transforms' key found.");
    return 0;
  }

  for (const auto & item : config["static_transforms"]) {
    try {
      std::string parent = item["parent"].as<std::string>();
      std::string child  = item["child"].as<std::string>();
      double x     = item["xyz"][0].as<double>();
      double y     = item["xyz"][1].as<double>();
      double z     = item["xyz"][2].as<double>();
      double roll  = item["rpy"][0].as<double>();
      double pitch = item["rpy"][1].as<double>();
      double yaw   = item["rpy"][2].as<double>();

      transforms_.push_back(buildTransform(parent, child, x, y, z, roll, pitch, yaw));

      RCLCPP_INFO(node_->get_logger(),
        "[StaticTfManager] Loaded: %s -> %s  xyz[%.3f,%.3f,%.3f] rpy[%.3f,%.3f,%.3f]",
        parent.c_str(), child.c_str(), x, y, z, roll, pitch, yaw);

    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "[StaticTfManager] Parse error: %s", e.what());
    }
  }

  if (!transforms_.empty()) {
    static_broadcaster_->sendTransform(transforms_);
    RCLCPP_INFO(node_->get_logger(),
      "[StaticTfManager] Published %zu static transforms.", transforms_.size());
  }

  return static_cast<int>(transforms_.size());
}

void StaticTfManager::addTransform(const geometry_msgs::msg::TransformStamped & tf)
{
  transforms_.push_back(tf);
  static_broadcaster_->sendTransform(transforms_);
  RCLCPP_INFO(node_->get_logger(),
    "[StaticTfManager] Added: %s -> %s",
    tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
}

geometry_msgs::msg::TransformStamped StaticTfManager::buildTransform(
  const std::string & parent, const std::string & child,
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp    = node_->get_clock()->now();
  ts.header.frame_id = parent;
  ts.child_frame_id  = child;

  ts.transform.translation.x = x;
  ts.transform.translation.y = y;
  ts.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();

  return ts;
}

}  // namespace tf_manager