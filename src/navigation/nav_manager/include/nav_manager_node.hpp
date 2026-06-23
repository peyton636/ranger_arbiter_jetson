#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "nav2/simple_navigate_executor.hpp"
#include "safety/lidar_safety_monitor.hpp"
#include "types/nav_types.hpp"
#include "visibility_control.hpp"

namespace navigation
{

/// @brief 导航管理 Lifecycle 节点
///
/// 职责：对 Nav2 做一层业务封装，不让任务层直接依赖 Nav2 细节。
/// - 订阅 task_fsm 的 /goal_pose
/// - 订阅 /lidar/object (LaserScan) 与相机/感知数据，做安全门控
/// - use_nav2=true：转发目标给 Nav2，中继 Nav2 速度到底盘
/// - use_nav2=false：内置 SimpleNavigateExecutor 直接输出 cmd_vel
class NAV_MANAGER_PUBLIC NavManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NavManagerNode(const rclcpp::NodeOptions & options);
  ~NavManagerNode() override = default;

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declareParameters();
  LidarSafetyConfig loadSafetyConfig();
  SimpleNavigateConfig loadSimpleNavConfig();

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onObjectPoses(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void onNav2CmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  void onNavResult(NavActionStatus status, const std::string & message);
  void publishArrived(bool arrived);
  void publishState();
  void publishChassisCmd(const geometry_msgs::msg::Twist & cmd);
  void onTick();

  std::string nav_goal_topic_;
  std::string nav_arrived_topic_;
  std::string nav2_goal_topic_;
  std::string lidar_topic_;
  std::string object_pose_topic_;
  std::string camera_info_topic_;
  std::string nav2_cmd_vel_topic_;
  std::string chassis_cmd_vel_topic_;
  std::string current_pose_topic_;
  std::string nav_state_topic_;
  std::string odom_topic_;
  bool use_nav2_{true};
  bool publish_state_{true};

  LidarSafetyConfig safety_config_;
  SimpleNavigateConfig simple_nav_config_;
  std::unique_ptr<LidarSafetyMonitor> lidar_safety_;
  std::unique_ptr<SimpleNavigateExecutor> simple_nav_;

  NavState nav_state_{NavState::IDLE};
  geometry_msgs::msg::PoseStamped active_goal_;
  nav_msgs::msg::Odometry latest_odom_;
  bool has_active_goal_{false};
  bool has_odom_{false};

  mutable std::mutex mutex_;
  geometry_msgs::msg::PoseArray latest_object_poses_;
  bool has_object_poses_{false};
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool has_camera_info_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr object_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace navigation
