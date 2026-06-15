#ifndef CONVERSION__SEGMENTATION_CONVERTER_HPP_
#define CONVERSION__SEGMENTATION_CONVERTER_HPP_

#include <vector>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "adapters/yolos_adapter_base.hpp"
#include "visibility_control.hpp"

namespace perception {
namespace conversion {

/// @brief Convert SegmentationResults to Detection2DArray (boxes only)
PERCEPTION_YOLO_PUBLIC
vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<SegmentationResult>& segmentations,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

/// @brief Convert mask to sensor_msgs::msg::Image
PERCEPTION_YOLO_PUBLIC
sensor_msgs::msg::Image toMaskImage(
  const cv::Mat& mask,
  const std_msgs::msg::Header& header);

/// @brief Combine all masks into single multi-class mask image
PERCEPTION_YOLO_PUBLIC
sensor_msgs::msg::Image toCombinedMaskImage(
  const std::vector<SegmentationResult>& segmentations,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

}  // namespace conversion
}  // namespace perception

#endif  // CONVERSION__SEGMENTATION_CONVERTER_HPP_
