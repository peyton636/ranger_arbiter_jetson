#include "nav_manager_node.hpp"

#include <cmath>

namespace navigation {

NavManagerNode::NavManagerNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("nav_manager_node", options) {
  declareParameters();
}

void NavManagerNode::declareParameters() {
  nav_goal_topic_ =
      declare_parameter<std::string>("nav_goal_topic", "/goal_pose");
  nav_arrived_topic_ =
      declare_parameter<std::string>("nav_arrived_topic", "/task/nav_arrived");
  nav2_goal_topic_ =
      declare_parameter<std::string>("nav2_goal_topic", "/nav_goal_pose");

  lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/lidar/object");
  object_pose_topic_ = declare_parameter<std::string>(
      "object_pose_topic", "/perception/tracked_object_pose_array");
  camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/color/camera_info");

  nav2_cmd_vel_topic_ =
      declare_parameter<std::string>("nav2_cmd_vel_topic", "/cmd_vel_nav2");
  chassis_cmd_vel_topic_ =
      declare_parameter<std::string>("chassis_cmd_vel_topic", "/cmd_vel");
  current_pose_topic_ =
      declare_parameter<std::string>("current_pose_topic", "/nav/current_pose");
  nav_state_topic_ =
      declare_parameter<std::string>("nav_state_topic", "/nav/state");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");

  use_nav2_ = declare_parameter<bool>("use_nav2", false);
  publish_state_ = declare_parameter<bool>("publish_state", true);

  safety_config_.stop_distance =
      declare_parameter<double>("stop_distance", 0.35);
  safety_config_.slow_distance =
      declare_parameter<double>("slow_distance", 0.60);
  safety_config_.slow_scale = declare_parameter<double>("slow_scale", 0.4);
  safety_config_.front_angle_deg =
      declare_parameter<double>("front_angle_deg", 60.0);
  safety_config_.min_valid_range =
      declare_parameter<double>("min_valid_range", 0.10);

  simple_nav_config_.goal_tolerance_xy =
      declare_parameter<double>("goal_tolerance_xy", 0.10);
  simple_nav_config_.goal_tolerance_yaw =
      declare_parameter<double>("goal_tolerance_yaw", 0.15);
  simple_nav_config_.max_linear_vel =
      declare_parameter<double>("max_linear_vel", 0.3);
  simple_nav_config_.max_angular_vel =
      declare_parameter<double>("max_angular_vel", 0.8);
  simple_nav_config_.linear_gain =
      declare_parameter<double>("linear_gain", 0.8);
  simple_nav_config_.angular_gain =
      declare_parameter<double>("angular_gain", 1.5);
}

LidarSafetyConfig NavManagerNode::loadSafetyConfig() {
  LidarSafetyConfig cfg;
  cfg.stop_distance = get_parameter("stop_distance").as_double();
  cfg.slow_distance = get_parameter("slow_distance").as_double();
  cfg.slow_scale = get_parameter("slow_scale").as_double();
  cfg.front_angle_deg = get_parameter("front_angle_deg").as_double();
  cfg.min_valid_range = get_parameter("min_valid_range").as_double();
  return cfg;
}

SimpleNavigateConfig NavManagerNode::loadSimpleNavConfig() {
  SimpleNavigateConfig cfg;
  cfg.goal_tolerance_xy = get_parameter("goal_tolerance_xy").as_double();
  cfg.goal_tolerance_yaw = get_parameter("goal_tolerance_yaw").as_double();
  cfg.max_linear_vel = get_parameter("max_linear_vel").as_double();
  cfg.max_angular_vel = get_parameter("max_angular_vel").as_double();
  cfg.linear_gain = get_parameter("linear_gain").as_double();
  cfg.angular_gain = get_parameter("angular_gain").as_double();
  return cfg;
}

NavManagerNode::CallbackReturn
NavManagerNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring NavManagerNode...");
  safety_config_ = loadSafetyConfig();
  simple_nav_config_ = loadSimpleNavConfig();
  lidar_safety_ = std::make_unique<LidarSafetyMonitor>(safety_config_);
  simple_nav_ = std::make_unique<SimpleNavigateExecutor>(simple_nav_config_);
  simple_nav_->setResultCallback(
      [this](NavActionStatus status, const std::string &message) {
        onNavResult(status, message);
      });

  chassis_cmd_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(chassis_cmd_vel_topic_, 10);
  nav2_goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(nav2_goal_topic_, 10);
  arrived_pub_ = create_publisher<std_msgs::msg::Bool>(nav_arrived_topic_, 10);
  current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      current_pose_topic_, 10);
  if (publish_state_) {
    state_pub_ = create_publisher<std_msgs::msg::String>(nav_state_topic_, 10);
  }

  RCLCPP_INFO(get_logger(), "Configured: goal=%s nav2_goal=%s use_nav2=%s",
              nav_goal_topic_.c_str(), nav2_goal_topic_.c_str(),
              use_nav2_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

NavManagerNode::CallbackReturn
NavManagerNode::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating NavManagerNode...");

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      nav_goal_topic_, 10,
      std::bind(&NavManagerNode::onGoal, this, std::placeholders::_1));

  lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NavManagerNode::onLaserScan, this, std::placeholders::_1));

  object_pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      object_pose_topic_, 10,
      std::bind(&NavManagerNode::onObjectPoses, this, std::placeholders::_1));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&NavManagerNode::onCameraInfo, this, std::placeholders::_1));

  nav2_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav2_cmd_vel_topic_, 10,
      std::bind(&NavManagerNode::onNav2CmdVel, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&NavManagerNode::onOdom, this, std::placeholders::_1));

  chassis_cmd_pub_->on_activate();
  nav2_goal_pub_->on_activate();
  arrived_pub_->on_activate();
  current_pose_pub_->on_activate();
  if (state_pub_) {
    state_pub_->on_activate();
  }

  tick_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                  std::bind(&NavManagerNode::onTick, this));

  publishArrived(false);
  publishState();
  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

NavManagerNode::CallbackReturn
NavManagerNode::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating NavManagerNode...");
  tick_timer_.reset();
  goal_sub_.reset();
  lidar_sub_.reset();
  object_pose_sub_.reset();
  camera_info_sub_.reset();
  nav2_cmd_vel_sub_.reset();
  odom_sub_.reset();

  geometry_msgs::msg::Twist stop;
  publishChassisCmd(stop);

  chassis_cmd_pub_->on_deactivate();
  nav2_goal_pub_->on_deactivate();
  arrived_pub_->on_deactivate();
  current_pose_pub_->on_deactivate();
  if (state_pub_) {
    state_pub_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

NavManagerNode::CallbackReturn
NavManagerNode::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up NavManagerNode...");
  tick_timer_.reset();
  goal_sub_.reset();
  lidar_sub_.reset();
  object_pose_sub_.reset();
  camera_info_sub_.reset();
  nav2_cmd_vel_sub_.reset();
  odom_sub_.reset();
  chassis_cmd_pub_.reset();
  nav2_goal_pub_.reset();
  arrived_pub_.reset();
  current_pose_pub_.reset();
  state_pub_.reset();

  if (simple_nav_) {
    simple_nav_->reset();
  }
  simple_nav_.reset();
  lidar_safety_.reset();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_active_goal_ = false;
    has_object_poses_ = false;
    has_camera_info_ = false;
    has_odom_ = false;
  }
  nav_state_ = NavState::IDLE;
  return CallbackReturn::SUCCESS;
}

NavManagerNode::CallbackReturn
NavManagerNode::on_shutdown(const rclcpp_lifecycle::State &state) {
  return on_cleanup(state);
}

void NavManagerNode::onGoal(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_goal_ = *msg;
    has_active_goal_ = true;
  }

  nav_state_ = NavState::NAVIGATING;
  publishState();
  publishArrived(false);

  RCLCPP_INFO(get_logger(), "Received nav goal (%.2f, %.2f) frame=%s",
              msg->pose.position.x, msg->pose.position.y,
              msg->header.frame_id.c_str());

  if (use_nav2_) {
    if (nav2_goal_pub_ && nav2_goal_pub_->is_activated()) {
      nav2_goal_pub_->publish(*msg);
    }
    return;
  }

  if (!simple_nav_ || !simple_nav_->sendGoal(*msg)) {
    nav_state_ = NavState::FAILED;
    publishState();
    publishArrived(false);
    RCLCPP_ERROR(get_logger(), "Failed to start simple navigation");
  }
}

void NavManagerNode::onLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!msg || !lidar_safety_) {
    return;
  }
  lidar_safety_->updateScan(*msg);

  if (lidar_safety_->isBlocked() && nav_state_ == NavState::NAVIGATING) {
    nav_state_ = NavState::BLOCKED;
    publishState();
  } else if (!lidar_safety_->isBlocked() && nav_state_ == NavState::BLOCKED) {
    nav_state_ = NavState::NAVIGATING;
    publishState();
  }
}

void NavManagerNode::onObjectPoses(
    const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_object_poses_ = *msg;
  has_object_poses_ = true;
}

void NavManagerNode::onCameraInfo(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_camera_info_ = *msg;
  has_camera_info_ = true;
}

void NavManagerNode::onNav2CmdVel(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!msg || !lidar_safety_ || !use_nav2_) {
    return;
  }

  if (nav_state_ != NavState::NAVIGATING && nav_state_ != NavState::BLOCKED) {
    return;
  }

  const auto safe_cmd = lidar_safety_->filterCmdVel(*msg);
  publishChassisCmd(safe_cmd);
}

void NavManagerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!msg) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  if (current_pose_pub_ && current_pose_pub_->is_activated()) {
    current_pose_pub_->publish(pose);
  }
}

void NavManagerNode::onNavResult(NavActionStatus status,
                                 const std::string &message) {
  RCLCPP_INFO(get_logger(), "Navigation result: %s (%s)",
              navActionStatusToString(status).c_str(), message.c_str());

  if (status == NavActionStatus::SUCCEEDED) {
    nav_state_ = NavState::ARRIVED;
    publishArrived(true);
  } else {
    nav_state_ = NavState::FAILED;
    publishArrived(false);
  }
  publishState();

  geometry_msgs::msg::Twist stop;
  publishChassisCmd(stop);
}

void NavManagerNode::publishArrived(bool arrived) {
  if (!arrived_pub_ || !arrived_pub_->is_activated()) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = arrived;
  arrived_pub_->publish(msg);
}

void NavManagerNode::publishState() {
  if (!state_pub_ || !state_pub_->is_activated()) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = navStateToString(nav_state_);
  state_pub_->publish(msg);
}

void NavManagerNode::publishChassisCmd(const geometry_msgs::msg::Twist &cmd) {
  if (!chassis_cmd_pub_ || !chassis_cmd_pub_->is_activated()) {
    return;
  }
  chassis_cmd_pub_->publish(cmd);
}

void NavManagerNode::onTick() {
  if (nav_state_ == NavState::BLOCKED && lidar_safety_ &&
      !lidar_safety_->isBlocked()) {
    nav_state_ = NavState::NAVIGATING;
    publishState();
  }

  if (use_nav2_ || !simple_nav_ || !lidar_safety_) {
    // Nav2 模式下根据里程计判断到达（Nav2 不直接连 task_fsm）
    if (use_nav2_ && nav_state_ == NavState::NAVIGATING) {
      nav_msgs::msg::Odometry odom;
      geometry_msgs::msg::PoseStamped goal;
      bool ready = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_odom_ && has_active_goal_) {
          odom = latest_odom_;
          goal = active_goal_;
          ready = true;
        }
      }
      if (ready) {
        const double dx = goal.pose.position.x - odom.pose.pose.position.x;
        const double dy = goal.pose.position.y - odom.pose.pose.position.y;
        if (std::hypot(dx, dy) <= simple_nav_config_.goal_tolerance_xy) {
          nav_state_ = NavState::ARRIVED;
          publishArrived(true);
          publishState();
          geometry_msgs::msg::Twist stop;
          publishChassisCmd(stop);
        }
      }
    }
    return;
  }

  nav_msgs::msg::Odometry odom;
  bool has_odom = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_odom_) {
      odom = latest_odom_;
      has_odom = true;
    }
  }

  if (!has_odom || nav_state_ != NavState::NAVIGATING) {
    return;
  }

  const auto cmd = simple_nav_->update(odom);
  if (!cmd.has_value()) {
    return;
  }

  if (nav_state_ == NavState::BLOCKED) {
    geometry_msgs::msg::Twist stop;
    publishChassisCmd(stop);
    return;
  }

  publishChassisCmd(lidar_safety_->filterCmdVel(cmd.value()));
}

} // namespace navigation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navigation::NavManagerNode)
