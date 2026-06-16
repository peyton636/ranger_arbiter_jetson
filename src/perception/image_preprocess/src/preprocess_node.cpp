#include "preprocess_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

namespace image_preprocess
{

    PreprocessNode::PreprocessNode() : Node("image_preprocess_node")
    {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/camera/color/image_raw");
        camera_info_yaml_ = this->declare_parameter<std::string>("camera_params");

        // ROI 参数：
        // 如果 width/height 为 0，则表示使用整幅去畸变后的图像
        roi_x_ = this->declare_parameter<int>("roi.x", 0);
        roi_y_ = this->declare_parameter<int>("roi.y", 0);
        roi_w_ = this->declare_parameter<int>("roi.width", 0);  // 0 -> full width
        roi_h_ = this->declare_parameter<int>("roi.height", 0); // 0 -> full height

        // 不同推理模型名称
        model_names_ = this->declare_parameter<std::vector<std::string>>(
            "model_names", std::vector<std::string>{"detector", "obb", "pose", "segmentor"});

        // 不同推理模型需要的输入宽度\高度
        model_widths_ = this->declare_parameter<std::vector<int64_t>>(
            "model_widths", std::vector<int64_t>{640, 1024, 640, 640});
        model_heights_ = this->declare_parameter<std::vector<int64_t>>(
            "model_heights", std::vector<int64_t>{640, 1024, 640, 640});

        if (model_names_.size() != model_widths_.size() || model_names_.size() != model_heights_.size())
        {
            throw std::runtime_error("model_names/model_widths/model_heights size mismatch");
        }

        loadCameraYaml(camera_info_yaml_);

        //为每个模型创建一个输出图像 publisher
        for (size_t i = 0; i < model_names_.size(); ++i)
        {
            OutputTarget t;
            t.name = model_names_[i];
            t.width = static_cast<int>(model_widths_[i]);
            t.height = static_cast<int>(model_heights_[i]);
            const std::string topic = "/preprocess/" + t.name + "/image";
            t.pub = this->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::SensorDataQoS());
            outputs_.push_back(t);
            RCLCPP_INFO(this->get_logger(), "Output: %s -> %dx%d (%s)",
                        t.name.c_str(), t.width, t.height, topic.c_str());
        }

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PreprocessNode::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "image_preprocess_node started.");
    }

    void PreprocessNode::loadCameraYaml(const std::string &path)
    {
        YAML::Node root = YAML::LoadFile(path);

        // 读取内参矩阵 k（3x3，展开成长度 9 的数组）
        const auto k = root["k"].as<std::vector<double>>();

        // 读取畸变参数 d
        const auto d = root["d"].as<std::vector<double>>();

        // 基本合法性检查
        if (k.size() != 9)
        {
            throw std::runtime_error("camera_info.yaml 'k' must have 9 elements");
        }
        if (d.empty())
        {
            throw std::runtime_error("camera_info.yaml 'd' is empty");
        }

        // 构造 OpenCV 相机内参矩阵
        K_ = (cv::Mat_<double>(3, 3) << 
              k[0], k[1], k[2],
              k[3], k[4], k[5],
              k[6], k[7], k[8]);

        // 构造 OpenCV 畸变系数矩阵
        D_ = cv::Mat(1, static_cast<int>(d.size()), CV_64F);
        for (size_t i = 0; i < d.size(); ++i)
        {
            D_.at<double>(0, static_cast<int>(i)) = d[i];
        }

        RCLCPP_INFO(this->get_logger(), "Loaded camera yaml: %s", path.c_str());
    }

    // 初始化去畸变映射表：
    // 只有当输入图像尺寸变化，或 map 尚未生成时，才重新计算
    void PreprocessNode::initUndistortMapIfNeeded(int img_w, int img_h)
    {
        if (img_w == map_w_ && img_h == map_h_ && !map1_.empty() && !map2_.empty())
        {
            return;
        }

        // 单目场景下，R 通常使用单位阵
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

        // 新相机矩阵，这里直接沿用原始 K
        // 含义：尽量保持原始视场角和主点
        cv::Mat new_K = K_.clone(); // keep same FOV/principal point

        // 预计算 remap 所需的映射表，后续每帧直接 remap，效率更高
        cv::initUndistortRectifyMap(
            K_, D_, R, new_K, cv::Size(img_w, img_h), CV_32FC1, map1_, map2_);

        map_w_ = img_w;
        map_h_ = img_h;
    }

    // 计算安全 ROI：
    // 防止 ROI 越界；如果配置非法，则退回整幅图像
    cv::Rect PreprocessNode::safeRoi(int img_w, int img_h) const
    {
        int x = std::max(0, roi_x_);
        int y = std::max(0, roi_y_);

        // 如果 roi.width/height <= 0，则默认用剩余整幅区域
        int w = (roi_w_ <= 0) ? (img_w - x) : roi_w_;
        int h = (roi_h_ <= 0) ? (img_h - y) : roi_h_;

        w = std::min(w, img_w - x);
        h = std::min(h, img_h - y);

        // 如果最终 ROI 无效，则返回整幅图像
        if (w <= 0 || h <= 0)
        {
            return cv::Rect(0, 0, img_w, img_h);
        }
        return cv::Rect(x, y, w, h);
    }

    // 图像回调：
    // 原始图像 -> 去畸变 -> ROI 裁剪 -> 归一化 -> resize 到不同模型尺寸 -> 发布
    void PreprocessNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;

        // ROS Image 转 OpenCV Mat，强制转成 bgr8
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        const cv::Mat &raw = cv_ptr->image;
        if (raw.empty())
        {
            return;
        }

        // 根据当前图像尺寸，初始化或更新去畸变映射表
        initUndistortMapIfNeeded(raw.cols, raw.rows);

        // 去畸变
        cv::Mat undistorted;
        cv::remap(raw, undistorted, map1_, map2_, cv::INTER_LINEAR);

        // 裁剪 ROI
        cv::Rect roi = safeRoi(undistorted.cols, undistorted.rows);
        cv::Mat cropped = undistorted(roi).clone();

        // normalize to [0, 1], float32
        // 输出格式为 CV_32FC3，便于后续推理前处理
        cv::Mat normalized;
        cropped.convertTo(normalized, CV_32FC3, 1.0 / 255.0);

        // 为每个目标模型生成对应尺寸的输入图像
        for (auto &out : outputs_)
        {
            // 直接 resize 到目标尺寸
            cv::Mat resized;
            cv::resize(normalized, resized, cv::Size(out.width, out.height), 0, 0, cv::INTER_LINEAR);

            // 发布 32FC3 格式图像
            auto out_msg = cv_bridge::CvImage(msg->header, "32FC3", resized).toImageMsg();
            out.pub->publish(*out_msg);
        }
    }

} // namespace image_preprocess