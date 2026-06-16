#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

namespace image_preprocess
{

struct OutputTarget
{
  std::string name;
  int width;
  int height;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
};

class PreprocessNode : public rclcpp::Node
{
public:
  PreprocessNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void loadCameraYaml(const std::string & path);
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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::vector<OutputTarget> outputs_;

  // Camera
  cv::Mat K_;
  cv::Mat D_;
  cv::Mat map1_;
  cv::Mat map2_;
  int map_w_{0};
  int map_h_{0};
};

}  // namespace image_preprocess