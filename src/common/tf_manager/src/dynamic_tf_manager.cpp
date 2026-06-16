#include "tf_manager/dynamic_tf_manager.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace tf_manager
{

DynamicTfManager::DynamicTfManager(rclcpp::Node::SharedPtr node)
: node_(node)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

  node_->declare_parameter("odom_timeout_sec",  1.0);
  node_->declare_parameter("joint_timeout_sec", 2.0);
  node_->declare_parameter("tf_max_delay_sec",  0.1);
  node_->declare_parameter("odom_topic",        std::string("/odom"));
  node_->declare_parameter("joint_topic",       std::string("/joint_states"));

  odom_timeout_sec_  = node_->get_parameter("odom_timeout_sec").as_double();
  joint_timeout_sec_ = node_->get_parameter("joint_timeout_sec").as_double();
  tf_max_delay_sec_  = node_->get_parameter("tf_max_delay_sec").as_double();

  std::string odom_topic  = node_->get_parameter("odom_topic").as_string();
  std::string joint_topic = node_->get_parameter("joint_topic").as_string();

  watch_chains_ = {
    {"map",       "base_link"},
    {"odom",      "base_link"},
    {"base_link", "camera_link"},
    {"base_link", "arm_base_link"},
  };

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 20,
    std::bind(&DynamicTfManager::odomCallback, this, std::placeholders::_1));

  joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_topic, 20,
    std::bind(&DynamicTfManager::jointStateCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "[DynamicTfManager] Subscribed odom=%s  joints=%s",
    odom_topic.c_str(), joint_topic.c_str());
}

void DynamicTfManager::startMonitor()
{
  monitor_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&DynamicTfManager::monitorCallback, this));
  RCLCPP_INFO(node_->get_logger(), "[DynamicTfManager] Monitor started (500ms).");
}

void DynamicTfManager::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_time_ = node_->get_clock()->now();
  odom_received_  = true;

  geometry_msgs::msg::TransformStamped ts;
  ts.header         = msg->header;
  ts.child_frame_id = msg->child_frame_id.empty() ? "base_link" : msg->child_frame_id;

  ts.transform.translation.x = msg->pose.pose.position.x;
  ts.transform.translation.y = msg->pose.pose.position.y;
  ts.transform.translation.z = msg->pose.pose.position.z;
  ts.transform.rotation      = msg->pose.pose.orientation;

  tf_broadcaster_->sendTransform(ts);
}

void DynamicTfManager::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr /*msg*/)
{
  last_joint_time_ = node_->get_clock()->now();
  joint_received_  = true;
}

void DynamicTfManager::monitorCallback()
{
  auto now = node_->get_clock()->now();

  // 里程计超时
  if (odom_received_) {
    double elapsed = (now - last_odom_time_).seconds();
    if (elapsed > odom_timeout_sec_) {
      RCLCPP_WARN(node_->get_logger(),
        "[TfMonitor] ODOM timeout! No data for %.2fs (max=%.2fs)",
        elapsed, odom_timeout_sec_);
    }
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[TfMonitor] Waiting for first /odom message...");
  }

  // 关节状态超时
  if (joint_received_) {
    double elapsed = (now - last_joint_time_).seconds();
    if (elapsed > joint_timeout_sec_) {
      RCLCPP_WARN(node_->get_logger(),
        "[TfMonitor] JointState timeout! No data for %.2fs (max=%.2fs)",
        elapsed, joint_timeout_sec_);
    }
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[TfMonitor] Waiting for first /joint_states message...");
  }

  // TF 链路检查
  for (const auto & [src, tgt] : watch_chains_) {
    checkTfChain(src, tgt, tf_max_delay_sec_);
  }
}

void DynamicTfManager::checkTfChain(
  const std::string & source,
  const std::string & target,
  double max_delay_sec)
{
  if (!tf_buffer_->canTransform(target, source, tf2::TimePointZero)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "[TfMonitor] BROKEN: %s -> %s", source.c_str(), target.c_str());
    return;
  }

  try {
    auto ts = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
    double delay = (node_->get_clock()->now() - rclcpp::Time(ts.header.stamp)).seconds();

    if (delay > max_delay_sec) {
      RCLCPP_WARN(node_->get_logger(),
        "[TfMonitor] HIGH LATENCY: %s -> %s  delay=%.3fs (max=%.3fs)",
        source.c_str(), target.c_str(), delay, max_delay_sec);
    } else {
      RCLCPP_DEBUG(node_->get_logger(),
        "[TfMonitor] OK: %s -> %s  delay=%.3fs", source.c_str(), target.c_str(), delay);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "[TfMonitor] Exception %s -> %s: %s", source.c_str(), target.c_str(), ex.what());
  }
}

}  // namespace tf_manager