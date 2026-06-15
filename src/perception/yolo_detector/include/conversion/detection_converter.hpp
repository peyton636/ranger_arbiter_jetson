#ifndef CONVERSION__DETECTION_CONVERTER_HPP_
#define CONVERSION__DETECTION_CONVERTER_HPP_

#include <vector>
#include <string>

#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_msgs/msg/header.hpp>

#include "adapters/yolos_adapter_base.hpp"
#include "visibility_control.hpp"

namespace perception {
namespace conversion {

/// @brief Convert DetectionResult to vision_msgs::msg::Detection2D
PERCEPTION_YOLO_PUBLIC
vision_msgs::msg::Detection2D toDetection2D(
  const DetectionResult& det,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

/// @brief Convert vector of DetectionResults to vision_msgs::msg::Detection2DArray
PERCEPTION_YOLO_PUBLIC
vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<DetectionResult>& detections,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

}  // namespace conversion
}  // namespace perception

#endif  // CONVERSION__DETECTION_CONVERTER_HPP_
