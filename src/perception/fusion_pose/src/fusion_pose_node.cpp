#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include "fusion_pose_node.hpp"
#include "rclcpp/qos.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <yaml-cpp/yaml.h>

namespace perception {

FusionPoseNode::FusionPoseNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("fusion_pose_node", options) {
  RCLCPP_INFO(get_logger(), "FusionPoseNode constructor called");
  declareParameters();
}

void FusionPoseNode::declareParameters() {
  // 2D 检测结果输入话题，要求发布 vision_msgs::msg::Detection2DArray
  // 消息，且消息中的 bbox.center 与 bbox.size_x/y 能正确反映检测框中心和尺寸
  detections_topic_ = this->declare_parameter<std::string>(
      "detections_topic", "/perception/detect/detections_2d");

  // 深度图输入话题，要求与 detections_topic_
  // 同步，且对应同一时间戳的消息能正确转换为 OpenCV 图像
  depth_topic_ = this->declare_parameter<std::string>(
      "depth_topic", "/camera/depth/image_raw");

  // 相机内参输入话题
  camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/camera/color/camera_info");

  // 输出的 3D 位姿数组话题，位姿相对于 target_frame_ 坐标系
  output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/perception/object_pose_array");

  // 目标坐标系，空字符串表示直接输出相机坐标系下结果
  target_frame_ = this->declare_parameter<std::string>("target_frame", "");

  // 手眼标定 YAML 文件路径
  hand_eye_yaml_ = this->declare_parameter<std::string>("params_file", "");

  // 深度采样窗口大小
  depth_window_ = this->declare_parameter<int>("depth_window", 5);

  // 深度图单位缩放系数，例如毫米转米时取 0.001
  depth_scale_ = this->declare_parameter<double>("depth_scale", 0.001);

  // 有效深度最小值
  min_depth_ = this->declare_parameter<double>("min_depth", 0.1);

  // 有效深度最大值
  max_depth_ = this->declare_parameter<double>("max_depth", 5.0);

  // 检测框 ROI 采样比例
  bbox_roi_scale_ = this->declare_parameter<double>("bbox_roi_scale", 0.4);
}

FusionPoseNode::CallbackReturn
FusionPoseNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring FusionPoseNode...");
  // depth_window 必须为正数，否则使用默认值 5
  if (depth_window_ <= 0) {
    depth_window_ = 5;
  }

  // depth_window 最好是奇数，便于以中心点对称取窗口
  if (depth_window_ % 2 == 0) {
    depth_window_ += 1;
  }

  // 限制 bbox_roi_scale 在合理范围内，避免 ROI 过小或过大导致深度估计不稳定
  bbox_roi_scale_ = std::clamp(bbox_roi_scale_, 0.1, 1.0);

  // 输出目标位姿数组，消息类型 geometry_msgs::msg::PoseArray，位姿相对于
  // target_frame_ 坐标系
  if (!pose_pub_)
    pose_pub_ = this->create_publisher<PoseArrayMsg>(output_topic_, 10);

  // TF 缓冲区和监听器，用于查询坐标变换
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 如果配置了手眼标定文件，则加载固定外参
  if (!hand_eye_yaml_.empty()) {
    loadHandEyeYaml(hand_eye_yaml_);
  }
  return CallbackReturn::SUCCESS;
}

FusionPoseNode::CallbackReturn
FusionPoseNode::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating FusionPoseNode...");
  // 订阅 2D 检测、深度图、相机内参，并进行时间同步
  const rmw_qos_profile_t rmw_qos_profile_sensor_data =
      rclcpp::SensorDataQoS().get_rmw_qos_profile();

  const rmw_qos_profile_t camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                                                .reliable()
                                                .durability_volatile()
                                                .get_rmw_qos_profile();

  detections_sub_.subscribe(this, detections_topic_,
                            rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
  camera_info_sub_.subscribe(this, camera_info_topic_, camera_info_qos);

  // 创建同步器，保证三路消息尽量按同一时间戳进入回调
  sync_ = std::make_shared<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<DetectionArray, ImageMsg,
                                                      CameraInfoMsg>>>(
      message_filters::sync_policies::ApproximateTime<DetectionArray, ImageMsg,
                                                      CameraInfoMsg>(20),
      detections_sub_, depth_sub_, camera_info_sub_);

  sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(0.1)); // 设置最大时间间隔，超过则不匹配
  RCLCPP_INFO(get_logger(), "sync created, max interval = 0.1s");
  // 注册同步回调，当 detections_msg、depth_msg、camera_info_msg
  // 三者在时间上近似同步时被调用
  sync_->registerCallback(
      std::bind(&FusionPoseNode::syncCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(this->get_logger(), "fusion_pose_node started");
  RCLCPP_INFO(this->get_logger(), "detections_topic: %s",
              detections_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "depth_topic: %s", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "camera_info_topic: %s",
              camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "target_frame: %s",
              target_frame_.empty() ? "<camera_frame>" : target_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

FusionPoseNode::CallbackReturn
FusionPoseNode::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating FusionPoseNode...");
  try {
    // 先停同步器，避免回调继续进入
    sync_.reset();

    // 取消三路订阅
    detections_sub_.unsubscribe();
    depth_sub_.unsubscribe();
    camera_info_sub_.unsubscribe();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "on_deactivate failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

FusionPoseNode::CallbackReturn
FusionPoseNode::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up FusionPoseNode...");
  try {
    // 与 on_deactivate 一样先停回调链路
    sync_.reset();
    detections_sub_.unsubscribe();
    depth_sub_.unsubscribe();
    camera_info_sub_.unsubscribe();

    // 释放通信与 TF 资源
    tf_listener_.reset();
    tf_buffer_.reset();

    // 清理 hand-eye 缓存
    handeye_parent_frame_.clear();
    handeye_child_frame_.clear();
    handeye_parent_T_camera_.setIdentity();
    has_handeye_ = false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "on_cleanup failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

FusionPoseNode::CallbackReturn
FusionPoseNode::on_shutdown(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(),
              "Shutting down FusionPoseNode... [state id=%u, label=%s]",
              state.id(), state.label().c_str());

  // 关闭阶段按“先停订阅，再释放资源”执行
  auto ret = on_deactivate(state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  return on_cleanup(state);
}

// 同步回调：输入检测框、深度图、相机内参，输出目标位姿数组
void FusionPoseNode::syncCallback(
    const DetectionArray::ConstSharedPtr &detections_msg,
    const ImageMsg::ConstSharedPtr &depth_msg,
    const CameraInfoMsg::ConstSharedPtr &camera_info_msg) {
  // RCLCPP_INFO(this->get_logger(), "syncCallback called with %zu detections",
  // detections_msg->detections.size()); 从相机内参中读取 fx, fy, cx, cy
  const double fx = camera_info_msg->k[0];
  const double fy = camera_info_msg->k[4];
  const double cx = camera_info_msg->k[2];
  const double cy = camera_info_msg->k[5];

  // 内参非法时直接退出
  if (fx <= 0.0 || fy <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid camera intrinsics");
    return;
  }

  // 将 ROS 深度图消息转换为 OpenCV 图像，支持多种编码格式
  cv_bridge::CvImageConstPtr depth_cv_ptr;
  try {
    depth_cv_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat &depth_image = depth_cv_ptr->image;
  if (depth_image.empty()) {
    return;
  }

  // 确定当前相机坐标系名称
  std::string camera_frame = camera_info_msg->header.frame_id;
  if (camera_frame.empty()) {
    camera_frame = depth_msg->header.frame_id;
  }
  if (camera_frame.empty()) {
    camera_frame = handeye_child_frame_;
  }

  // 准备输出位姿数组
  PoseArrayMsg pose_array;
  pose_array.header.stamp = detections_msg->header.stamp;
  pose_array.header.frame_id =
      target_frame_.empty() ? camera_frame : target_frame_;

  // RCLCPP_INFO(this->get_logger(), "Processing %zu detections with camera
  // frame '%s'", detections_msg->detections.size(), camera_frame.c_str());
  // 遍历每个检测目标，估计其三维位置
  for (const auto &det : detections_msg->detections) {
    double depth_m = 0.0;

    // 从检测框对应区域中提取一个代表深度值
    if (!getDepthFromDetection(depth_image, det, depth_m)) {
      continue;
    }

    // 检测框中心像素坐标
    const double u = det.bbox.center.x;
    const double v = det.bbox.center.y;

    // 根据针孔相机模型把像素坐标反投影到相机坐标系
    const double x = (u - cx) * depth_m / fx;
    const double y = (v - cy) * depth_m / fy;
    const double z = depth_m;

    tf2::Vector3 p_camera(x, y, z);
    tf2::Vector3 p_target;

    // 将相机坐标下的点变换到目标坐标系
    if (!transformPointToTarget(p_camera, detections_msg->header.stamp,
                                camera_frame, p_target)) {
      continue;
    }

    // RCLCPP_INFO(this->get_logger(),
    //             "class ID %s Detection at pixel (%.1f, %.1f) with depth %.3f
    //             m "
    //             "-> 3D point (%.3f, %.3f, %.3f) in target frame",
    //             det.class_id.c_str(), u, v, depth_m, p_target.x(),
    //             p_target.y(), p_target.z());
    // 组装输出位姿，当前只填位置，姿态设为单位四元数
    geometry_msgs::msg::Pose pose;
    pose.position.x = p_target.x();
    pose.position.y = p_target.y();
    pose.position.z = p_target.z();
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    pose_array.poses.push_back(pose);
  }

  // 发布结果
  //   RCLCPP_INFO(this->get_logger(), "Publishing %zu poses",
  //               pose_array.poses.size());
  pose_pub_->publish(pose_array);
}

// 读取手眼标定 YAML，得到机器人末端到相机的固定外参
void FusionPoseNode::loadHandEyeYaml(const std::string &yaml_path) {
  YAML::Node root = YAML::LoadFile(yaml_path);

  // 读取父子坐标系名称
  handeye_parent_frame_ = root["parent_frame"].as<std::string>();
  handeye_child_frame_ = root["child_frame"].as<std::string>();

  // 读取平移
  const auto t_x = root["translation_x"].as<double>();
  const auto t_y = root["translation_y"].as<double>();
  const auto t_z = root["translation_z"].as<double>();
  // const auto t = root["translation"];

  // // 读取旋转四元数
  const auto q_x = root["rotation_quaternion_x"].as<double>();
  const auto q_y = root["rotation_quaternion_y"].as<double>();
  const auto q_z = root["rotation_quaternion_z"].as<double>();
  const auto q_w = root["rotation_quaternion_w"].as<double>();
  // const auto q = root["rotation_quaternion"];

  // const double tx = t["x"].as<double>();
  // const double ty = t["y"].as<double>();
  // const double tz = t["z"].as<double>();

  // const double qx = q["x"].as<double>();
  // const double qy = q["y"].as<double>();
  // const double qz = q["z"].as<double>();
  // const double qw = q["w"].as<double>();

  // 四元数归一化，避免数值误差导致变换不正确
  tf2::Quaternion quat(q_x, q_y, q_z, q_w);
  quat.normalize();

  // 保存 hand-eye 外参：parent_frame -> camera_frame
  handeye_parent_T_camera_ = tf2::Transform(quat, tf2::Vector3(t_x, t_y, t_z));
  has_handeye_ = true;

  RCLCPP_INFO(this->get_logger(), "Loaded hand-eye yaml: %s",
              yaml_path.c_str());
  RCLCPP_INFO(this->get_logger(), "hand-eye transform: %s <- %s",
              handeye_parent_frame_.c_str(), handeye_child_frame_.c_str());
}

// 从检测框对应区域中提取一个稳定深度值
bool FusionPoseNode::getDepthFromDetection(
    const cv::Mat &depth_image, const vision_msgs::msg::Detection2D &det,
    double &depth_m) const {
  if (depth_image.empty()) {
    return false;
  }

  // 检测框中心与尺寸
  const double center_u = det.bbox.center.x;
  const double center_v = det.bbox.center.y;
  const double box_w = det.bbox.size_x;
  const double box_h = det.bbox.size_y;

  // 中心点非法则直接失败
  if (!std::isfinite(center_u) || !std::isfinite(center_v)) {
    return false;
  }

  // 保存所有有效深度值，后面用中位数抑制噪声
  std::vector<double> valid_depths;
  valid_depths.reserve(256);

  // 统一的取深度函数：支持 16UC1 和 32FC1
  auto push_depth = [&](int x, int y) {
    // 越界直接跳过
    if (x < 0 || y < 0 || x >= depth_image.cols || y >= depth_image.rows) {
      return;
    }

    double d = 0.0;

    // 16 位深度图，通常单位为毫米
    if (depth_image.type() == CV_16UC1) {
      const uint16_t raw = depth_image.at<uint16_t>(y, x);
      if (raw == 0U) {
        return;
      }
      d = static_cast<double>(raw) * depth_scale_;
    }

    // 32 位浮点深度图，通常单位为米
    else if (depth_image.type() == CV_32FC1) {
      const float raw = depth_image.at<float>(y, x);
      if (!std::isfinite(raw) || raw <= 0.0f) {
        return;
      }
      d = static_cast<double>(raw);
    } else {
      return;
    }

    // 只保留合法深度范围内的数据
    if (std::isfinite(d) && d >= min_depth_ && d <= max_depth_) {
      valid_depths.push_back(d);
    }
  };

  // 先在检测框区域内部采样
  if (box_w > 1.0 && box_h > 1.0) {
    // 按比例截取检测框内部 ROI
    const int roi_w =
        std::max(1, static_cast<int>(std::round(box_w * bbox_roi_scale_)));
    const int roi_h =
        std::max(1, static_cast<int>(std::round(box_h * bbox_roi_scale_)));
    const int x0 =
        std::max(0, static_cast<int>(std::round(center_u)) - roi_w / 2);
    const int y0 =
        std::max(0, static_cast<int>(std::round(center_v)) - roi_h / 2);
    const int x1 = std::min(depth_image.cols - 1, x0 + roi_w - 1);
    const int y1 = std::min(depth_image.rows - 1, y0 + roi_h - 1);

    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        push_depth(x, y);
      }
    }
  }

  // 如果检测框内没有有效深度，则退化为中心点附近的固定窗口采样
  if (valid_depths.empty()) {
    const int u = static_cast<int>(std::round(center_u));
    const int v = static_cast<int>(std::round(center_v));
    const int half = depth_window_ / 2;

    for (int y = std::max(0, v - half);
         y <= std::min(depth_image.rows - 1, v + half); ++y) {
      for (int x = std::max(0, u - half);
           x <= std::min(depth_image.cols - 1, u + half); ++x) {
        push_depth(x, y);
      }
    }
  }

  // 仍然没有有效深度则失败
  if (valid_depths.empty()) {
    return false;
  }

  // 使用中位数作为最终深度，降低异常值影响
  auto mid = valid_depths.begin() + valid_depths.size() / 2;
  std::nth_element(valid_depths.begin(), mid, valid_depths.end());
  depth_m = *mid;
  return true;
}

// 将相机坐标系下的点转换到目标坐标系
bool FusionPoseNode::transformPointToTarget(
    const tf2::Vector3 &p_camera, const builtin_interfaces::msg::Time &stamp,
    const std::string &camera_frame, tf2::Vector3 &p_target) const {
  // 如果没有指定目标坐标系，或目标坐标系就是相机坐标系，则直接返回
  if (target_frame_.empty() || target_frame_ == camera_frame) {
    p_target = p_camera;
    return true;
  }

  // 如果加载了手眼标定，并且当前相机坐标系与标定文件中的 child_frame 一致
  if (has_handeye_ && camera_frame == handeye_child_frame_) {
    // 先把点从相机系变换到机器人末端/父坐标系
    const tf2::Vector3 p_parent = handeye_parent_T_camera_ * p_camera;

    // 如果目标坐标系就是父坐标系，直接返回
    if (target_frame_ == handeye_parent_frame_) {
      p_target = p_parent;
      return true;
    }

    // 否则继续通过 TF 查询父坐标系到目标坐标系的变换
    try {
      const auto tf_target_parent = tf_buffer_->lookupTransform(
          target_frame_, handeye_parent_frame_, rclcpp::Time(stamp),
          rclcpp::Duration::from_seconds(0.1));

      const tf2::Transform target_T_parent =
          transformMsgToTf(tf_target_parent.transform);
      p_target = target_T_parent * p_parent;
      return true;
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "lookupTransform(%s <- %s) failed: %s",
                  target_frame_.c_str(), handeye_parent_frame_.c_str(),
                  e.what());
      return false;
    }
  }

  // 没有用手眼外参的情况下，直接通过 TF 从 camera_frame 变换到 target_frame_
  try {
    const auto tf_target_camera = tf_buffer_->lookupTransform(
        target_frame_, camera_frame, rclcpp::Time(stamp),
        rclcpp::Duration::from_seconds(0.1));

    const tf2::Transform target_T_camera =
        transformMsgToTf(tf_target_camera.transform);
    p_target = target_T_camera * p_camera;
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "lookupTransform(%s <- %s) failed: %s",
                target_frame_.c_str(), camera_frame.c_str(), e.what());
    return false;
  }
}

// 将 geometry_msgs::msg::Transform 转成 tf2::Transform
tf2::Transform
FusionPoseNode::transformMsgToTf(const geometry_msgs::msg::Transform &msg) {
  // 提取四元数并归一化
  tf2::Quaternion q(msg.rotation.x, msg.rotation.y, msg.rotation.z,
                    msg.rotation.w);
  q.normalize();

  // 提取平移
  tf2::Vector3 t(msg.translation.x, msg.translation.y, msg.translation.z);

  // 组合成 tf2 变换
  return tf2::Transform(q, t);
}

} // namespace perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception::FusionPoseNode)