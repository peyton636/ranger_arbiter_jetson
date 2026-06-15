#ifndef ADAPTERS__POSE_ADAPTER_HPP_
#define ADAPTERS__POSE_ADAPTER_HPP_

#include <memory>
#include <vector>
#include "adapters/yolos_adapter_base.hpp"

namespace perception {

class PERCEPTION_YOLO_PUBLIC PoseAdapter : public IPoseAdapter {
public:
  PoseAdapter();
  ~PoseAdapter() override;
  PoseAdapter(PoseAdapter&&) noexcept;
  PoseAdapter& operator=(PoseAdapter&&) noexcept;

  bool initialize(const YolosConfig& config) override;
  [[nodiscard]] bool isInitialized() const noexcept override;
  void shutdown() override;
  [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

  std::vector<PoseResult> detect(
    const cv::Mat& image,
    float conf_threshold,
    float nms_threshold) override;

  void drawPoses(
    cv::Mat& image,
    const std::vector<PoseResult>& poses,
    int kpt_radius,
    float kpt_threshold) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception

#endif  // ADAPTERS__POSE_ADAPTER_HPP_
