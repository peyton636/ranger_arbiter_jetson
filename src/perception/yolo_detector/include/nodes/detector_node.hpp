#ifndef NODES__DETECTOR_NODE_HPP_
#define NODES__DETECTOR_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "visibility_control.hpp"
#include "adapters/detector_adapter.hpp"

namespace perception
{

  class PERCEPTION_YOLO_PUBLIC YolosDetectorNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit YolosDetectorNode(const rclcpp::NodeOptions &options);
    ~YolosDetectorNode() override = default;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
    void declareParameters();
    YolosConfig loadConfig();

    std::unique_ptr<IDetectorAdapter> detector_;
    rclcpp::CallbackGroup::SharedPtr inference_cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;

    std::string model_path_, labels_path_, yolo_version_;
    bool use_gpu_, publish_debug_image_, publish_timing_;
    float conf_threshold_, nms_threshold_;
  };

} // namespace perception

#endif // NODES__DETECTOR_NODE_HPP_
