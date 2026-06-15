#ifndef CLASSIFIER_ADAPTER_HPP
#define CLASSIFIER_ADAPTER_HPP

#include <memory>
#include <vector>
#include "adapters/yolos_adapter_base.hpp"


namespace perception
{

    class PERCEPTION_YOLO_PUBLIC ClassifierAdapter : public IClassifierAdapter
    {
    public:
        ClassifierAdapter();
        ~ClassifierAdapter() override;
        ClassifierAdapter(ClassifierAdapter&&) noexcept;
        ClassifierAdapter& operator=(ClassifierAdapter&&) noexcept;

        bool initialize(const YolosConfig& config) override;
        [[nodiscard]] bool isInitialized() const noexcept override;
        void shutdown() override;
        [[nodiscard]] const std::vector<std::string>& getClassNames() const override;

        ClassificationResult classify(const cv::Mat& image) override;

        void drawResult(
            cv::Mat& image,
            const ClassificationResult& result) override;

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;

    };

}


#endif // CLASSIFIER_ADAPTER_HPP