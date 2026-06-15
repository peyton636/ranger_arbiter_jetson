#ifndef ADAPTERS__OBB_ADAPTER_HPP_
#define ADAPTERS__OBB_ADAPTER_HPP_

#include <memory>
#include <vector>
#include "adapters/yolos_adapter_base.hpp"

namespace perception {

class PERCEPTION_YOLO_PUBLIC OBBAdapter : public IOBBAdapter {
public:
  OBBAdapter();
  ~OBBAdapter() override;
  OBBAdapter(OBBAdapter&&) noexcept;
  OBBAdapter& operator=(OBBAdapter&&) noexcept;

  bool initialize(const YolosConfig& config) override;
  [[nodiscard]] bool isInitialized() const noexcept override;
  void shutdown() override;
  [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

  std::vector<OBBResult> detect(
    const cv::Mat& image,
    float conf_threshold,
    float nms_threshold,
    int max_det) override;

  void drawDetections(
    cv::Mat& image,
    const std::vector<OBBResult>& detections) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception

#endif  // ADAPTERS__OBB_ADAPTER_HPP_
