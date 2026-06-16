#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <vector>

namespace tf_manager
{

class StaticTfManager
{
public:
  explicit StaticTfManager(rclcpp::Node::SharedPtr node);

  int loadAndPublish(const std::string & yaml_path);

  void addTransform(const geometry_msgs::msg::TransformStamped & tf);

private:
  geometry_msgs::msg::TransformStamped buildTransform(
    const std::string & parent,
    const std::string & child,
    double x, double y, double z,
    double roll, double pitch, double yaw);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
};

}  // namespace tf_manager