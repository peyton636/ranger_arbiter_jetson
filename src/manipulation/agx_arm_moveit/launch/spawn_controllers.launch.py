"""激活 ros2_control 控制器（arm_controller、gripper_controller 等）。

由 demo.launch.py 在 ros2_control_node 启动后包含调用。
通过 controller_manager 的 spawner 加载并启动 JointTrajectoryController，
使 MoveIt Execute 能将规划轨迹发送到 control/joint_states。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from moveit_configs_utils.launches import generate_spawn_controllers_launch

from _moveit_config_builder import build_moveit_config, declare_common_args


def _launch(context):
    moveit_config = build_moveit_config(context)
    return list(generate_spawn_controllers_launch(moveit_config).entities)


def generate_launch_description():
    return LaunchDescription(declare_common_args() + [OpaqueFunction(function=_launch)])
