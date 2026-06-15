#ifndef CONVERSION__CLASSIFICATION_CONVERTER_HPP_
#define CONVERSION__CLASSIFICATION_CONVERTER_HPP_

#include <vision_msgs/msg/classification2_d.hpp>
#include <std_msgs/msg/header.hpp>

#include "adapters/yolos_adapter_base.hpp"
#include "visibility_control.hpp"

namespace perception {
namespace conversion {

/// @brief Convert ClassificationResult to vision_msgs::msg::Classification2D
PERCEPTION_YOLO_PUBLIC
vision_msgs::msg::Classification2D toClassification2D(
  const ClassificationResult& result,
  const std_msgs::msg::Header& header);

}  // namespace conversion
}  // namespace perception

#endif  // CONVERSION__CLASSIFICATION_CONVERTER_HPP_
