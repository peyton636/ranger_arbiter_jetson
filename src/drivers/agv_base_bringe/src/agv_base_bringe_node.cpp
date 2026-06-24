#include "agv_base_bringe_node.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "jetson/jetson_protocol.hpp"
#include "jetson/mcu_time_sync.hpp"

namespace agv_base_driver
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

/// 兼容 Nav2（volatile）与 ros2 topic pub 默认（transient_local）的 cmd_vel 订阅
rclcpp::QoS cmdVelQoS()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.transient_local();
  return qos;
}

/// 兼容 Nav2（volatile）与 GUI（transient_local）的 /agv_control 订阅
rclcpp::QoS agvControlQoS()
{
  return cmdVelQoS();
}

}  // namespace

AgvBaseBringeNode::AgvBaseBringeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("agv_base_bringe", options)
{
  declareParameters();
}

void AgvBaseBringeNode::declareParameters()
{
  link_type_ = declare_parameter<std::string>("link_type", "rs232");
  agv_control_topic_ = declare_parameter<std::string>("agv_control_topic", "/agv_control");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  cmd_vel_compat_enable_ = declare_parameter<bool>("cmd_vel_compat_enable", true);
  feature_status_topic_ = declare_parameter<std::string>(
    "feature_status_topic", "/Function/FeatureStatusInfo");
  vehicle_topic_ = declare_parameter<std::string>("vehicle_topic", "/Vehicle/VehicleData");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 50.0);
  status_timeout_s_ =
    declare_parameter<int>("status_timeout_ms", 500) / 1000.0;
  cmd_timeout_s_ =
    declare_parameter<int>("cmd_timeout_ms", kCmdTimeoutMs) / 1000.0;
  max_linear_m_s_ = declare_parameter<double>("max_linear_m_s", 0.8);
  max_angular_rad_s_ = declare_parameter<double>("max_angular_rad_s", 1.0);
  cruise_scale_ = declare_parameter<double>("cruise_scale", 1.0);
  mode_req_ = static_cast<uint8_t>(declare_parameter<int>("mode_req", kModeCan));
  pending_mode_req_ = mode_req_;
  strafe_jl_from_angular_ = declare_parameter<bool>("strafe_jl_from_angular", true);
  strafe_speed_m_s_ = declare_parameter<double>("strafe_speed_m_s", 0.3);
  sideways_steer_millirad_ =
    declare_parameter<int>("sideways_steer_millirad", 1571);

  jetson_prefix_ = "/jetson_" + link_type_;
  command_topic_ = jetson_prefix_ + "/command";
  status_topic_ = jetson_prefix_ + "/v3_status";
  ext_status_topic_ = jetson_prefix_ + "/v3_ext_status";
  motion_blob_topic_ = jetson_prefix_ + "/blob/motion";
  time_sync_topic_ = jetson_prefix_ + "/time_sync";
}

AgvBaseBringeNode::CallbackReturn
AgvBaseBringeNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring AgvBaseBringeNode...");

  // configure 阶段创建 publisher，activate 后再订阅与定时器
  command_pub_ = create_publisher<jetson_mcu_msgs::msg::V3Command>(command_topic_, 10);
  vehicle_pub_ = create_publisher<scr_sensor::msg::VehicleData>(vehicle_topic_, 10);
  feature_pub_ = create_publisher<scr_sensor::msg::AgvFeatureStatus>(feature_status_topic_, 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  RCLCPP_INFO(
    get_logger(),
    "Configured: link=%s %s → %s | %s, %s, %s",
    link_type_.c_str(), agv_control_topic_.c_str(), command_topic_.c_str(),
    vehicle_topic_.c_str(), feature_status_topic_.c_str(), odom_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

AgvBaseBringeNode::CallbackReturn
AgvBaseBringeNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating AgvBaseBringeNode...");

  agv_control_sub_ = create_subscription<scr_sensor::msg::AgvControl>(
    agv_control_topic_, agvControlQoS(),
    std::bind(&AgvBaseBringeNode::onAgvControl, this, std::placeholders::_1));
  if (cmd_vel_compat_enable_) {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, cmdVelQoS(),
      std::bind(&AgvBaseBringeNode::onCmdVel, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "cmd_vel 兼容已启用：%s → 内部 AgvControl 语义",
      cmd_vel_topic_.c_str());
  }
  status_sub_ = create_subscription<jetson_mcu_msgs::msg::V3Status>(
    status_topic_, 10,
    std::bind(&AgvBaseBringeNode::onStatus, this, std::placeholders::_1));
  ext_sub_ = create_subscription<jetson_mcu_msgs::msg::V3ExtStatus>(
    ext_status_topic_, 10,
    std::bind(&AgvBaseBringeNode::onExtStatus, this, std::placeholders::_1));
  motion_blob_sub_ = create_subscription<jetson_mcu_msgs::msg::BlobAgvMotion>(
    motion_blob_topic_, 10,
    std::bind(&AgvBaseBringeNode::onMotionBlob, this, std::placeholders::_1));
  time_sync_sub_ = create_subscription<jetson_mcu_msgs::msg::TimeSyncResponse>(
    time_sync_topic_, 10,
    std::bind(&AgvBaseBringeNode::onTimeSync, this, std::placeholders::_1));

  const double period = cmd_rate_hz_ > 0.0 ? (1.0 / cmd_rate_hz_) : 0.02;
  // 定时器 1：周期性下发 V3Command（即使 cmd_vel 为空也发零速心跳）
  cmd_timer_ = create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&AgvBaseBringeNode::publishCommand, this));
  // 定时器 2：融合状态并发布 odom/TF
  state_timer_ = create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&AgvBaseBringeNode::publishState, this));

  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

AgvBaseBringeNode::CallbackReturn
AgvBaseBringeNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating AgvBaseBringeNode...");
  cmd_timer_.reset();
  state_timer_.reset();
  agv_control_sub_.reset();
  cmd_vel_sub_.reset();
  status_sub_.reset();
  ext_sub_.reset();
  motion_blob_sub_.reset();
  time_sync_sub_.reset();
  return CallbackReturn::SUCCESS;
}

AgvBaseBringeNode::CallbackReturn
AgvBaseBringeNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up AgvBaseBringeNode...");
  cmd_timer_.reset();
  state_timer_.reset();
  agv_control_sub_.reset();
  cmd_vel_sub_.reset();
  status_sub_.reset();
  ext_sub_.reset();
  motion_blob_sub_.reset();
  time_sync_sub_.reset();

  command_pub_.reset();
  vehicle_pub_.reset();
  feature_pub_.reset();
  odom_pub_.reset();
  tf_broadcaster_.reset();

  std::lock_guard<std::mutex> lock(mutex_);
  has_status_ = false;
  has_ext_ = false;
  pending_v_mm_s_ = 0;
  pending_omega_ = 0;
  pending_steer_ = 0;
  pending_motion_model_ = 0;
  recover_until_ = 0.0;
  last_odom_time_.reset();
  offset_valid_ = false;
  return CallbackReturn::SUCCESS;
}

AgvBaseBringeNode::CallbackReturn
AgvBaseBringeNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  const auto ret = on_deactivate(state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  return on_cleanup(state);
}

double AgvBaseBringeNode::steadyNowSec()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void AgvBaseBringeNode::applyMotionCommand(const MotionCommand & motion)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_v_mm_s_ = motion.v_mm_s;
  pending_omega_ = motion.omega_millirad_s;
  pending_steer_ = motion.steer_millirad;
  pending_motion_model_ = motion.motion_model;
  last_cmd_time_ = steadyNowSec();

  if (motion.v_mm_s != 0 || motion.omega_millirad_s != 0 || motion.steer_millirad != 0) {
    last_motion_v_mm_s_ = pending_v_mm_s_;
    last_motion_omega_ = pending_omega_;
    last_motion_steer_ = pending_steer_;
  }
}

void AgvBaseBringeNode::applyAgvControlFields(const scr_sensor::msg::AgvControl & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_mode_req_ = msg.mode_req;
  light_enable_ = msg.light_enable != 0 ? msg.light_enable : 1;
  light_mode_ = msg.light_mode;
  if (msg.clear_error != 0) {
    pending_clear_error_ = msg.clear_error;
  }
}

void AgvBaseBringeNode::onAgvControl(const scr_sensor::msg::AgvControl::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  applyAgvControlFields(*msg);

  MotionCommand motion;
  if (msg->use_twist_input) {
    double lx = msg->linear_x * cruise_scale_;
    double ly = msg->linear_y * cruise_scale_;
    double az = msg->angular_z * cruise_scale_;
    lx = std::clamp(lx, -max_linear_m_s_, max_linear_m_s_);
    ly = std::clamp(ly, -max_linear_m_s_, max_linear_m_s_);
    az = std::clamp(az, -max_angular_rad_s_, max_angular_rad_s_);
    motion = twistToMotion(
      lx, ly, az, strafe_jl_from_angular_, strafe_speed_m_s_, sideways_steer_millirad_);
  } else {
    motion.v_mm_s = msg->v_mm_s;
    motion.omega_millirad_s = msg->omega_millirad_s;
    motion.steer_millirad = msg->steer_millirad;
    motion.motion_model = msg->motion_model;
  }

  applyMotionCommand(motion);

  if (motion.v_mm_s != 0 || motion.omega_millirad_s != 0 || motion.steer_millirad != 0) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "agv_control → v=%d mm/s omega=%d steer=%d",
      motion.v_mm_s, motion.omega_millirad_s, motion.steer_millirad);
  }
}

void AgvBaseBringeNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  double lx = msg->linear.x * cruise_scale_;
  double ly = msg->linear.y * cruise_scale_;
  double az = msg->angular.z * cruise_scale_;
  lx = std::clamp(lx, -max_linear_m_s_, max_linear_m_s_);
  ly = std::clamp(ly, -max_linear_m_s_, max_linear_m_s_);
  az = std::clamp(az, -max_angular_rad_s_, max_angular_rad_s_);

  const auto motion = twistToMotion(
    lx, ly, az, strafe_jl_from_angular_, strafe_speed_m_s_, sideways_steer_millirad_);
  applyMotionCommand(motion);

  if (motion.v_mm_s != 0 || motion.omega_millirad_s != 0 || motion.steer_millirad != 0) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cmd_vel(compat) → v=%d mm/s omega=%d steer=%d",
      motion.v_mm_s, motion.omega_millirad_s, motion.steer_millirad);
  }
}

void AgvBaseBringeNode::onMotionBlob(
  const jetson_mcu_msgs::msg::BlobAgvMotion::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_motion_ = *msg;
  has_motion_blob_ = true;
}

void AgvBaseBringeNode::onStatus(const jetson_mcu_msgs::msg::V3Status::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_status_ = *msg;
  has_status_ = true;
  last_status_time_ = steadyNowSec();
}

void AgvBaseBringeNode::onExtStatus(const jetson_mcu_msgs::msg::V3ExtStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ext_ = *msg;
  has_ext_ = true;
}

void AgvBaseBringeNode::onTimeSync(
  const jetson_mcu_msgs::msg::TimeSyncResponse::SharedPtr msg)
{
  if (!msg || !msg->offset_valid) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  offset_ms_ = msg->offset_ms;
  offset_valid_ = true;
}

bool AgvBaseBringeNode::inRecovery() const
{
  return steadyNowSec() < recover_until_;
}

void AgvBaseBringeNode::startRecovery()
{
  // 进入 1s 零速窗口，清空 pending 运动
  recover_until_ = steadyNowSec() + kRecoverStableMs / 1000.0;
  pending_v_mm_s_ = 0;
  pending_omega_ = 0;
  pending_steer_ = 0;
  pending_motion_model_ = 0;
}

AgvBaseBringeNode::ResolvedMotion AgvBaseBringeNode::resolveMotion()
{
  ResolvedMotion out;
  const double now = steadyNowSec();

  // 拷贝共享状态，缩短持锁时间
  int16_t pending_v = 0;
  int16_t pending_omega = 0;
  int16_t pending_steer = 0;
  uint8_t pending_model = 0;
  double last_cmd = 0.0;
  int16_t last_v = 0;
  int16_t last_omega = 0;
  int16_t last_steer = 0;
  double recover_until = 0.0;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_v = pending_v_mm_s_;
    pending_omega = pending_omega_;
    pending_steer = pending_steer_;
    pending_model = pending_motion_model_;
    last_cmd = last_cmd_time_;
    last_v = last_motion_v_mm_s_;
    last_omega = last_motion_omega_;
    last_steer = last_motion_steer_;
    recover_until = recover_until_;
  }

  const bool cmd_stale = (now - last_cmd) > cmd_timeout_s_;
  const bool had_recent_motion = (last_v != 0 || last_omega != 0 || last_steer != 0);

  // 曾在运动但 cmd_vel 已超时 → 触发零速恢复
  if (had_recent_motion && cmd_stale && now >= recover_until) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (now >= recover_until_) {
      startRecovery();
      recover_until = recover_until_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "控制指令超时，进入 1s 零速恢复");
    }
  }

  const bool recovering = now < recover_until;
  const bool allow_motion = !recovering && !cmd_stale &&
    (pending_v != 0 || pending_omega != 0 || pending_steer != 0);

  if (recovering) {
    if (now >= recover_until) {
      std::lock_guard<std::mutex> lock(mutex_);
      recover_until_ = 0.0;
      if (cmd_stale) {
        // 恢复结束且 cmd_vel 仍超时 → 清除运动记忆，保持 IDLE
        last_motion_v_mm_s_ = 0;
        last_motion_omega_ = 0;
        last_motion_steer_ = 0;
      } else {
        RCLCPP_INFO(get_logger(), "恢复完成，继续执行控制指令");
      }
    }
    out.mode = "RECOVER";
    return out;
  }

  if (!allow_motion) {
    out.mode = "IDLE";
    return out;
  }

  out.v_mm_s = pending_v;
  out.omega_millirad_s = pending_omega;
  out.steer_millirad = pending_steer;
  out.motion_model = pending_model;
  out.mode = "RUN";
  return out;
}

void AgvBaseBringeNode::publishCommand()
{
  if (!command_pub_) {
    return;
  }

  const auto motion = resolveMotion();

  double offset_ms = 0.0;
  bool offset_valid = false;
  uint8_t light_enable = 1;
  uint8_t light_mode = 0;
  uint8_t mode_req = mode_req_;
  uint8_t clear_error = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    offset_ms = offset_ms_;
    offset_valid = offset_valid_;
    light_enable = light_enable_;
    light_mode = light_mode_;
    mode_req = pending_mode_req_;
    clear_error = pending_clear_error_;
    pending_clear_error_ = 0;
  }

  jetson_mcu_msgs::msg::V3Command msg;
  msg.header.stamp = mcuVirtualToRosStamp(offset_ms, offset_valid, *get_clock());
  msg.header.frame_id = base_frame_;
  msg.seq = 0;
  msg.mode_req = mode_req;
  msg.v_mm_s = motion.v_mm_s;
  msg.omega_millirad_s = motion.omega_millirad_s;
  msg.steer_millirad = motion.steer_millirad;
  msg.motion_model = motion.motion_model;
  msg.light_enable = light_enable;
  msg.light_mode = light_mode;
  msg.clear_error = clear_error;
  command_pub_->publish(msg);
}

std::optional<scr_sensor::msg::VehicleData> AgvBaseBringeNode::buildVehicleData() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_status_) {
    return std::nullopt;
  }

  const auto & s = latest_status_;
  scr_sensor::msg::VehicleData msg;
  msg.header = s.header;
  // --- 运动反馈（V3Status 0x102）---
  msg.linear_velocity_mm_s = s.fb_v_mm_s;
  msg.angular_velocity_millirad_s = s.fb_omega_millirad_s;
  msg.steer_millirad = s.fb_steer_millirad;
  msg.sonar_front_mm = s.sonar_front_mm;
  msg.sonar_back_mm = s.sonar_back_mm;
  msg.sonar_left_mm = s.sonar_left_mm;
  msg.sonar_right_mm = s.sonar_right_mm;
  msg.battery_voltage_0p1v = s.battery_voltage_0p1v;
  msg.battery_soc_percent = s.battery_soc;
  msg.v3_status_seq = s.seq;
  msg.data_valid = (steadyNowSec() - last_status_time_) <= status_timeout_s_;

  if (has_ext_) {
    // --- 底盘扩展（V3ExtStatus 0x103 最新一帧）---
    const auto & e = latest_ext_;
    msg.wheel_rf = e.wheel_rf;
    msg.wheel_rr = e.wheel_rr;
    msg.wheel_lr = e.wheel_lr;
    msg.wheel_lf = e.wheel_lf;
    msg.motor_temp_max_c = e.motor_temp_max_c;
    msg.driver_state_or = e.driver_state_or;
    msg.v3_ext_status_seq = e.seq;
  }
  return msg;
}

std::optional<scr_sensor::msg::AgvFeatureStatus> AgvBaseBringeNode::buildFeatureStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_status_) {
    return std::nullopt;
  }

  const auto & s = latest_status_;
  scr_sensor::msg::AgvFeatureStatus msg;
  msg.header = s.header;
  msg.safety_state = s.safety_state;
  msg.link_state = s.link_state;
  msg.limit_factor = s.limit_factor;
  msg.v3_status_seq = s.seq;
  msg.data_valid = (steadyNowSec() - last_status_time_) <= status_timeout_s_;

  if (has_motion_blob_) {
    msg.motion_model = latest_motion_.motion_info;
    msg.fault_code = latest_motion_.fault_code;
  } else {
    msg.fault_code = 0;
    const int abs_steer = std::abs(s.fb_steer_millirad);
    const int abs_omega = std::abs(s.fb_omega_millirad_s);
    const int abs_v = std::abs(s.fb_v_mm_s);
    if (abs_steer >= 800) {
      msg.motion_model = jetson_mcu_msgs::msg::V3Command::MOTION_SIDEWAYS;
    } else if (abs_omega >= 25 && abs_v < 15) {
      msg.motion_model = jetson_mcu_msgs::msg::V3Command::MOTION_SPIN;
    } else if (abs_v < 15 && abs_omega < 25) {
      msg.motion_model = jetson_mcu_msgs::msg::V3Command::MOTION_PARK;
    } else {
      msg.motion_model = jetson_mcu_msgs::msg::V3Command::MOTION_ACKERMANN;
    }
  }
  return msg;
}

void AgvBaseBringeNode::integrateOdom(const int16_t v_mm_s, const int16_t omega_millirad_s)
{
  const double now = steadyNowSec();
  if (!last_odom_time_.has_value()) {
    last_odom_time_ = now;
    return;
  }

  const double dt = now - *last_odom_time_;
  last_odom_time_ = now;
  // 跳变过大时跳过本帧，避免异常 dt 污染积分
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  // 差速模型简易积分：v 沿当前航向，omega 更新航向
  const double v = v_mm_s / 1000.0;
  const double omega = omega_millirad_s / 1000.0;
  odom_theta_ += omega * dt;
  odom_x_ += v * std::cos(odom_theta_) * dt;
  odom_y_ += v * std::sin(odom_theta_) * dt;
}

void AgvBaseBringeNode::publishState()
{
  if (!vehicle_pub_ || !odom_pub_) {
    return;
  }

  const auto vehicle_opt = buildVehicleData();
  if (!vehicle_opt.has_value()) {
    return;
  }

  const auto vehicle = *vehicle_opt;
  vehicle_pub_->publish(vehicle);

  if (feature_pub_) {
    const auto feature_opt = buildFeatureStatus();
    if (feature_opt.has_value()) {
      feature_pub_->publish(*feature_opt);
    }
  }

  if (vehicle.data_valid) {
    integrateOdom(vehicle.linear_velocity_mm_s, vehicle.angular_velocity_millirad_s);
  }

  // odom/TF 使用 v3_status 事件时间戳，勿用 wall now()
  const auto stamp = vehicle.header.stamp;
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.orientation.z = std::sin(odom_theta_ / 2.0);
  odom.pose.pose.orientation.w = std::cos(odom_theta_ / 2.0);
  odom.twist.twist.linear.x = vehicle.linear_velocity_mm_s / 1000.0;
  odom.twist.twist.angular.z = vehicle.angular_velocity_millirad_s / 1000.0;
  odom_pub_->publish(odom);

  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.rotation.z = std::sin(odom_theta_ / 2.0);
    tf.transform.rotation.w = std::cos(odom_theta_ / 2.0);
    tf_broadcaster_->sendTransform(tf);
  }
}

}  // namespace agv_base_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(agv_base_driver::AgvBaseBringeNode)
