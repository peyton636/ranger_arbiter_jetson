#ifndef CONVERSION__POSE_CONVERTER_HPP_
#define CONVERSION__POSE_CONVERTER_HPP_

#include <vector>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_msgs/msg/header.hpp>

#include "adapters/yolos_adapter_base.hpp"
#include "visibility_control.hpp"


namespace perception {
namespace conversion {

/// @brief Convert PoseResults to Detection2DArray with keypoint info in id/tracking_id
PERCEPTION_YOLO_PUBLIC
vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<PoseResult>& poses,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height);

}  // namespace conversion
}  // namespace perception

#endif  // CONVERSION__POSE_CONVERTER_HPP_
