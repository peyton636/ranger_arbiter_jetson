#include <memory>

#include "rclcpp/rclcpp.hpp"

class ObjectLocalizationNode : public rclcpp::Node
{
public:
  ObjectLocalizationNode()
  : Node("object_localization_node")
  {
    RCLCPP_INFO(this->get_logger(), "object_localization_node started");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
