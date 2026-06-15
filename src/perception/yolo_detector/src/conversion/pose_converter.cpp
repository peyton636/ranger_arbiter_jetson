#include "conversion/pose_converter.hpp"
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace perception {
namespace conversion {

vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<PoseResult>& poses,
  const std_msgs::msg::Header& header,
  int image_width,
  int image_height)
{
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;
  msg.detections.reserve(poses.size());

  // Suppress unused
  (void)image_width;
  (void)image_height;

  int detection_idx = 0;
  for (const auto& pose : poses) {
    vision_msgs::msg::Detection2D det;
    det.header = header;

    // Bounding box in center-based coordinates
    det.bbox.center.x = pose.bbox.x + pose.bbox.width / 2.0;
    det.bbox.center.y = pose.bbox.y + pose.bbox.height / 2.0;
    det.bbox.size_x = pose.bbox.width;
    det.bbox.size_y = pose.bbox.height;

    // Hypothesis
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.class_id = pose.class_name.empty() ? std::to_string(pose.class_id) : pose.class_name;
    hyp.score = pose.confidence;
    det.results.push_back(hyp);

    // Encode keypoints in id field as compact string (for downstream parsing)
    // Format: "pose_{idx}:kpt0_x,kpt0_y,kpt0_c;kpt1_x..."
    std::string kpt_data = "pose_" + std::to_string(detection_idx) + ":";
    for (size_t i = 0; i < pose.keypoints.size(); ++i) {
      if (i > 0) kpt_data += ";";
      kpt_data += std::to_string(static_cast<int>(pose.keypoints[i].x)) + "," +
                  std::to_string(static_cast<int>(pose.keypoints[i].y)) + "," +
                  std::to_string(pose.keypoints[i].confidence).substr(0, 4);
    }
    det.class_id = kpt_data;

    msg.detections.push_back(det);
    ++detection_idx;
  }

  return msg;
}

}  // namespace conversion
}  // namespace perception
