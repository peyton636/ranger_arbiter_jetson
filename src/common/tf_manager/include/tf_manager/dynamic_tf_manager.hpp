#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <vector>

namespace tf_manager
{

class DynamicTfManager
{
public:
  explicit DynamicTfManager(rclcpp::Node::SharedPtr node);
  ~DynamicTfManager() = default;

  void startMonitor();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void monitorCallback();
  void checkTfChain(
    const std::string & source,
    const std::string & target,
    double max_delay_sec);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  rclcpp::Time last_odom_time_;
  rclcpp::Time last_joint_time_;
  bool odom_received_{false};
  bool joint_received_{false};

  double odom_timeout_sec_{1.0};
  double joint_timeout_sec_{2.0};
  double tf_max_delay_sec_{0.1};

  std::vector<std::pair<std::string, std::string>> watch_chains_;
};

}  // namespace tf_manager