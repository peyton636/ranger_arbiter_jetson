#include "object_localization_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<object_localization::ObjectLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}