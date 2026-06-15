#include "nodes/classifier_node.hpp"
#include "conversion/classification_converter.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace perception {

YolosClassifierNode::YolosClassifierNode(const rclcpp::NodeOptions& o) 
: rclcpp_lifecycle::LifecycleNode("yolos_classifier", o) { 
  declareParameters(); 
}

void YolosClassifierNode::declareParameters() {
  RCLCPP_INFO(get_logger(), "===================================");
  declare_parameter("model_path", rclcpp::PARAMETER_STRING);
  declare_parameter("labels_path", rclcpp::PARAMETER_STRING);
  declare_parameter("use_gpu", false);
  declare_parameter("publish_debug_image", false);
}

YolosConfig YolosClassifierNode::loadConfig() {
  YolosConfig c;
  RCLCPP_INFO(get_logger(), "===================================");
  c.model_path = get_parameter("model_path").as_string();
  c.labels_path = get_parameter("labels_path").as_string();
  c.use_gpu = get_parameter("use_gpu").as_bool();
  debug_ = get_parameter("publish_debug_image").as_bool();
  return c;
}

YolosClassifierNode::CallbackReturn YolosClassifierNode::on_configure(const rclcpp_lifecycle::State&) {
  auto c = loadConfig();
  RCLCPP_INFO(get_logger(), "Configuring with model_path=%s, labels_path=%s, use_gpu=%s, publish_debug_image=%s",
    c.model_path.c_str(), c.labels_path.c_str(), c.use_gpu ? "true" : "false", debug_ ? "true" : "false");
  cls_ = createClassifierAdapter();
  if (!cls_->initialize(c)) return CallbackReturn::FAILURE;
  cb_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_ = create_publisher<vision_msgs::msg::Classification2D>("/perception/classify/classification2d", 10);
  if (debug_) dbg_ = create_publisher<sensor_msgs::msg::Image>("/perception/classify/debug_image", 1);
  return CallbackReturn::SUCCESS;
}

YolosClassifierNode::CallbackReturn YolosClassifierNode::on_activate(const rclcpp_lifecycle::State&) {
  pub_->on_activate(); if (dbg_) dbg_->on_activate();
  auto opt = rclcpp::SubscriptionOptions(); opt.callback_group = cb_;
  sub_ = create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&YolosClassifierNode::imageCallback, this, std::placeholders::_1), opt);
  return CallbackReturn::SUCCESS;
}

YolosClassifierNode::CallbackReturn YolosClassifierNode::on_deactivate(const rclcpp_lifecycle::State&) {
  sub_.reset(); pub_->on_deactivate(); if (dbg_) dbg_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

YolosClassifierNode::CallbackReturn YolosClassifierNode::on_cleanup(const rclcpp_lifecycle::State&) {
  if (cls_) { cls_->shutdown(); cls_.reset(); }
  pub_.reset(); dbg_.reset();
  return CallbackReturn::SUCCESS;
}

void YolosClassifierNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  if (!cls_ || !cls_->isInitialized()) return;
  try {
    auto cv = cv_bridge::toCvShare(msg, "bgr8");
    auto result = cls_->classify(cv->image);
    pub_->publish(perception::conversion::toClassification2D(result, msg->header));
    if (debug_ && dbg_ && dbg_->is_activated()) {
      cv::Mat d = cv->image.clone(); cls_->drawResult(d, result);
      dbg_->publish(*cv_bridge::CvImage(msg->header, "bgr8", d).toImageMsg());
    }
  } catch (const std::exception& e) { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", e.what()); }
}

}  // namespace perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception::YolosClassifierNode)
