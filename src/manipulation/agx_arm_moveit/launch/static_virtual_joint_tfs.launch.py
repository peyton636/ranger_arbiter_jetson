"""发布虚拟关节 TF（base_link 固定到 world 坐标系）。

由 demo.launch.py 包含调用。MoveIt 要求机器人根 link 相对 world 有 TF 定义，
本 launch 通过 static_transform_publisher 发布该固定变换。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

from _moveit_config_builder import build_moveit_config, declare_common_args


def _launch(context):
    moveit_config = build_moveit_config(context)
    return list(generate_static_virtual_joint_tfs_launch(moveit_config).entities)


def generate_launch_description():
    return LaunchDescription(declare_common_args() + [OpaqueFunction(function=_launch)])
