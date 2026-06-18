"""启动 RViz2，加载 MoveIt 运动规划插件（Plan / Execute 界面）。

由 demo.launch.py 在 use_rviz:=true 时包含调用。
加载 config/moveit.rviz，并注入 robot_description、规划管线、关节限位等参数。

rviz_config 参数由 demo.launch.py 动态生成（多臂时自动设置 Move Group Namespace）。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from _moveit_config_builder import build_moveit_config, declare_common_args


def _launch(context):
    follow = LaunchConfiguration("follow").perform(context)
    feedback_topic = LaunchConfiguration("feedback_topic").perform(context)
    control_topic = LaunchConfiguration("control_topic").perform(context)
    joint_states_topic = str(feedback_topic) if follow == "true" else str(control_topic)
    moveit_config = build_moveit_config(context)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            remappings=[("joint_states", joint_states_topic)],
        )
    ]


def generate_launch_description():
    return LaunchDescription(declare_common_args() + [OpaqueFunction(function=_launch)])
