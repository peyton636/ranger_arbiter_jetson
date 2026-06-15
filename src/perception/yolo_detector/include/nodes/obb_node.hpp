#ifndef NODES__OBB_NODE_HPP_
#define NODES__OBB_NODE_HPP_

#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "visibility_control.hpp"
#include "adapters/obb_adapter.hpp"
#include "vision_msgs/msg/obb_detection2_d_array.hpp"

namespace perception {

class PERCEPTION_YOLO_PUBLIC YolosOBBNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit YolosOBBNode(const rclcpp::NodeOptions& o);
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& s) override { return on_cleanup(s); }
private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr&);
  void declareParameters();
  YolosConfig loadConfig();
  std::unique_ptr<IOBBAdapter> obb_;
  rclcpp::CallbackGroup::SharedPtr cb_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::OBBDetection2DArray>::SharedPtr pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr dbg_;
  float conf_, nms_; int max_det_; bool debug_;
};

}  // namespace perception
#endif  // NODES__OBB_NODE_HPP_ 
