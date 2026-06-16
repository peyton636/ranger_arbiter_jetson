#include <rclcpp/rclcpp.hpp>
#include "tf_manager/static_tf_manager.hpp"
#include "tf_manager/dynamic_tf_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "tf_manager_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO(node->get_logger(), "===== TF Manager Node Starting =====");

  // YAML 路径参数
  node->declare_parameter(
    "static_tf_yaml",
    ament_index_cpp::get_package_share_directory("tf_manager") + "/config/static_tf.yaml");

  std::string yaml_path = node->get_parameter("static_tf_yaml").as_string();
  RCLCPP_INFO(node->get_logger(), "YAML path: %s", yaml_path.c_str());

  // 静态 TF
  auto static_mgr = std::make_shared<tf_manager::StaticTfManager>(node);
  if (std::filesystem::exists(yaml_path)) {
    int n = static_mgr->loadAndPublish(yaml_path);
    RCLCPP_INFO(node->get_logger(), "Loaded %d static transforms.", n);
  } else {
    RCLCPP_ERROR(node->get_logger(), "YAML not found: %s", yaml_path.c_str());
  }

  // 动态 TF
  auto dynamic_mgr = std::make_shared<tf_manager::DynamicTfManager>(node);
  dynamic_mgr->startMonitor();

  RCLCPP_INFO(node->get_logger(), "===== TF Manager Node Ready =====");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}