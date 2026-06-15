#ifndef CONVERSION__OBB_CONVERTER_HPP_
#define CONVERSION__OBB_CONVERTER_HPP_

#include <vector>
#include <std_msgs/msg/header.hpp>

#include "adapters/yolos_adapter_base.hpp"
#include "visibility_control.hpp"

// Forward declare custom message
#include "vision_msgs/msg/obb_detection2_d_array.hpp"

namespace perception {
namespace conversion {

/// @brief Convert OBBResults to custom OBBDetection2DArray message
PERCEPTION_YOLO_PUBLIC
vision_msgs::msg::OBBDetection2DArray toOBBDetection2DArray(
  const std::vector<OBBResult>& detections,
  const std_msgs::msg::Header& header);

}  // namespace conversion
}  // namespace perception

#endif  // CONVERSION__OBB_CONVERTER_HPP_
