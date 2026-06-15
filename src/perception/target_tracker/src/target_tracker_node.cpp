#include <memory>

#include "rclcpp/rclcpp.hpp"

class TargetTrackerNode : public rclcpp::Node
{
public:
  TargetTrackerNode()
  : Node("target_tracker_node")
  {
    RCLCPP_INFO(this->get_logger(), "target_tracker_node started");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
