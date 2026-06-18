#include "agx_gripper_ctrl_node.hpp"

namespace manipulation
{

  AgxGripperCtrlNode::AgxGripperCtrlNode(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("agx_gripper_ctrl_node", options)
  {
    RCLCPP_INFO(get_logger(), "AgxGripperCtrlNode constructor called");
  }


  AgxGripperCtrlNode::CallbackReturn AgxGripperCtrlNode::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring AgxGripperCtrlNode...");
    return CallbackReturn::SUCCESS;
  }

  AgxGripperCtrlNode::CallbackReturn AgxGripperCtrlNode::on_activate(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxGripperCtrlNode::CallbackReturn AgxGripperCtrlNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxGripperCtrlNode::CallbackReturn AgxGripperCtrlNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxGripperCtrlNode::CallbackReturn AgxGripperCtrlNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

} // namespace manipulation