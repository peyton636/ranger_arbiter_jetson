#ifndef ADAPTERS__DETECTOR_ADAPTER_HPP_
#define ADAPTERS__DETECTOR_ADAPTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "adapters/yolos_adapter_base.hpp"

// Forward declaration - actual YOLOs-CPP include only in .cpp
namespace yolos { namespace det { class YOLODetector; } }

namespace perception {

/// @brief Concrete detection adapter wrapping yolos::det::YOLODetector
class PERCEPTION_YOLO_PUBLIC DetectorAdapter : public IDetectorAdapter {
public:
  DetectorAdapter();
  ~DetectorAdapter() override;

  // Non-copyable, movable
  DetectorAdapter(const DetectorAdapter&) = delete;
  DetectorAdapter& operator=(const DetectorAdapter&) = delete;
  DetectorAdapter(DetectorAdapter&&) noexcept;
  DetectorAdapter& operator=(DetectorAdapter&&) noexcept;

  bool initialize(const YolosConfig& config) override;
  [[nodiscard]] bool isInitialized() const noexcept override;
  void shutdown() override;
  [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

  std::vector<DetectionResult> detect(
    const cv::Mat& image,
    float conf_threshold,
    float nms_threshold) override;

  void drawDetections(
    cv::Mat& image,
    const std::vector<DetectionResult>& detections) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception

#endif  // ADAPTERS__DETECTOR_ADAPTER_HPP_
