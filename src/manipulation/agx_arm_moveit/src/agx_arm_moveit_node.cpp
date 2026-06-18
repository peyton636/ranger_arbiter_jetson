#include "agx_arm_moveit_node.hpp"

namespace manipulation
{

  AgxArmMoveItNode::AgxArmMoveItNode(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("agx_arm_moveit_node", options)
  {
    RCLCPP_INFO(get_logger(), "AgxArmMoveItNode constructor called");
  }


  AgxArmMoveItNode::CallbackReturn AgxArmMoveItNode::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring AgxArmMoveItNode...");
    return CallbackReturn::SUCCESS;
  }

  AgxArmMoveItNode::CallbackReturn AgxArmMoveItNode::on_activate(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxArmMoveItNode::CallbackReturn AgxArmMoveItNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxArmMoveItNode::CallbackReturn AgxArmMoveItNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

  AgxArmMoveItNode::CallbackReturn AgxArmMoveItNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
      return CallbackReturn::SUCCESS;
  }

} // namespace manipulation