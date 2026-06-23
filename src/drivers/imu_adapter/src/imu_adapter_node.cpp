#include "imu_adapter_node.hpp"

namespace imu_adapter
{

ImuAdapterNode::ImuAdapterNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("imu_adapter", options)
{
  declareParameters();
}

ImuAdapterNode::~ImuAdapterNode()
{
  serial_.close();
}

void ImuAdapterNode::declareParameters()
{
  port_ = declare_parameter<std::string>("port", "/dev/imu_usb");
  baud_ = declare_parameter<int>("baud", 9600);
  frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
  mag_topic_ = declare_parameter<std::string>("mag_topic", "imu/mag");
  link_type_ = declare_parameter<std::string>("link_type", "eth");
  time_sync_topic_ = declare_parameter<std::string>("time_sync_topic", "");
  use_time_sync_stamp_ = declare_parameter<bool>("use_time_sync_stamp", true);
  poll_period_s_ = declare_parameter<double>("poll_period_s", 0.01);

  time_sync_.setUseTimeSync(use_time_sync_stamp_);

  if (time_sync_topic_.empty()) {
    time_sync_topic_ = "/jetson_" + link_type_ + "/time_sync";
  }
}

ImuAdapterNode::CallbackReturn
ImuAdapterNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring ImuAdapterNode...");

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_, 10);

  if (!serial_.open(port_, baud_)) {
    RCLCPP_ERROR(get_logger(), "无法打开 IMU 串口 %s", port_.c_str());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    get_logger(),
    "Configured: port=%s baud=%d imu=%s mag=%s time_sync=%s",
    port_.c_str(), baud_, imu_topic_.c_str(), mag_topic_.c_str(),
    use_time_sync_stamp_ ? time_sync_topic_.c_str() : "off");
  return CallbackReturn::SUCCESS;
}

ImuAdapterNode::CallbackReturn
ImuAdapterNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating ImuAdapterNode...");

  if (use_time_sync_stamp_) {
    time_sync_sub_ = create_subscription<jetson_mcu_msgs::msg::TimeSyncResponse>(
      time_sync_topic_, 10,
      std::bind(&ImuAdapterNode::onTimeSync, this, std::placeholders::_1));
  }

  poll_timer_ = create_wall_timer(
    std::chrono::duration<double>(poll_period_s_),
    std::bind(&ImuAdapterNode::pollSerial, this));

  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

ImuAdapterNode::CallbackReturn
ImuAdapterNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating ImuAdapterNode...");
  poll_timer_.reset();
  time_sync_sub_.reset();
  return CallbackReturn::SUCCESS;
}

ImuAdapterNode::CallbackReturn
ImuAdapterNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up ImuAdapterNode...");
  poll_timer_.reset();
  time_sync_sub_.reset();
  imu_pub_.reset();
  mag_pub_.reset();
  serial_.close();
  rx_buffer_.clear();
  wit_state_ = WitImuState{};
  return CallbackReturn::SUCCESS;
}

ImuAdapterNode::CallbackReturn
ImuAdapterNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  const auto ret = on_deactivate(state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  return on_cleanup(state);
}

void ImuAdapterNode::onTimeSync(
  const jetson_mcu_msgs::msg::TimeSyncResponse::SharedPtr msg)
{
  if (!msg || !msg->offset_valid) {
    return;
  }
  time_sync_.updateOffset(msg->offset_ms, true);
}

void ImuAdapterNode::pollSerial()
{
  if (!serial_.isOpen()) {
    return;
  }

  const auto chunk = serial_.readAvailable();
  if (chunk.empty()) {
    return;
  }

  rx_buffer_.insert(rx_buffer_.end(), chunk.begin(), chunk.end());

  // WIT 帧格式：0x55 + type + 8B data + checksum，共 11 字节
  while (rx_buffer_.size() >= 11) {
    if (rx_buffer_[0] != 0x55) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const std::vector<uint8_t> frame(rx_buffer_.begin(), rx_buffer_.begin() + 11);
    if (!verifyChecksum(frame.data(), frame.size())) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const bool publish_ready = feedFrame(frame, wit_state_);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 11);

    // 角度帧 0x53 到达时发布（与旧 wit_imu 行为一致）
    if (publish_ready) {
      publishImu(wit_state_);
    }
  }
}

void ImuAdapterNode::publishImu(const WitImuState & state)
{
  if (!imu_pub_ || !mag_pub_) {
    return;
  }

  const double sample_mono = state.sample_mono_ms.value_or(monoMs());
  const auto stamp = time_sync_.stampAtMono(sample_mono, *get_clock());

  sensor_msgs::msg::Imu imu_msg;
  sensor_msgs::msg::MagneticField mag_msg;
  fillImuMessage(state, frame_id_, stamp, imu_msg);
  fillMagMessage(state, frame_id_, stamp, mag_msg);

  imu_pub_->publish(imu_msg);
  mag_pub_->publish(mag_msg);
}

}  // namespace imu_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_adapter::ImuAdapterNode)
