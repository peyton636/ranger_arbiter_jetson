#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "visibility_control.hpp"

namespace perception
{

  struct OutputTarget
  {
    std::string name;
    int width;
    int height;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub;
  };

  class PERCEPTION_PREPROCESS_PUBLIC PreprocessNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit PreprocessNode(const rclcpp::NodeOptions &options);
    ~PreprocessNode() override = default;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void declareParameters();
    void loadCameraYaml(const std::string &path);
    void initUndistortMapIfNeeded(int img_w, int img_h);
    cv::Rect safeRoi(int img_w, int img_h) const;

  private:
    // Parameters
    std::string input_topic_;
    std::string camera_info_yaml_;
    int roi_x_;
    int roi_y_;
    int roi_w_;
    int roi_h_;
    std::vector<std::string> model_names_;
    std::vector<int64_t> model_widths_;
    std::vector<int64_t> model_heights_;

    // ROS
    rclcpp::CallbackGroup::SharedPtr inference_cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
    std::vector<OutputTarget> outputs_;

    // Camera
    cv::Mat K_;
    cv::Mat D_;
    cv::Mat map1_;
    cv::Mat map2_;
    int map_w_{0};
    int map_h_{0};

    bool publish_timing_;
  };

} // namespace perception