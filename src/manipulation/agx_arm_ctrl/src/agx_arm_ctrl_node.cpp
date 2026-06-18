#include "agx_arm_ctrl_node.hpp"

namespace manipulation
{

  AgxArmCtrlNode::AgxArmCtrlNode(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("agx_arm_ctrl_node", options)
  {
    RCLCPP_INFO(get_logger(), "AgxArmCtrlNode constructor called");
  }


  AgxArmCtrlNode::CallbackReturn AgxArmCtrlNode::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring AgxArmCtrlNode...");
    return CallbackReturn::SUCCESS;
  }

  AgxArmCtrlNode::CallbackReturn AgxArmCtrlNode::on_activate(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxArmCtrlNode::CallbackReturn AgxArmCtrlNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxArmCtrlNode::CallbackReturn AgxArmCtrlNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxArmCtrlNode::CallbackReturn AgxArmCtrlNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

} // namespace manipulation