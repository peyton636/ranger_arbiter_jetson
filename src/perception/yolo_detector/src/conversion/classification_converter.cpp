#include "conversion/classification_converter.hpp"
#include <vision_msgs/msg/object_hypothesis.hpp>

namespace perception {
namespace conversion {

vision_msgs::msg::Classification2D toClassification2D(
  const ClassificationResult& result,
  const std_msgs::msg::Header& header)
{
  vision_msgs::msg::Classification2D msg;
  msg.header = header;

  vision_msgs::msg::ObjectHypothesis hypothesis;
  hypothesis.class_id = result.class_name.empty() ? 
    std::to_string(result.class_id) : result.class_name;
  hypothesis.score = result.confidence;
  msg.results.push_back(hypothesis);

  return msg;
}

}  // namespace conversion
}  // namespace perception
