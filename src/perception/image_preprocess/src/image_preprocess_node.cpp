#include <memory>

#include "rclcpp/rclcpp.hpp"

class ImagePreprocessNode : public rclcpp::Node
{
public:
  ImagePreprocessNode()
  : Node("image_preprocess_node")
  {
    RCLCPP_INFO(this->get_logger(), "image_preprocess_node started");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
