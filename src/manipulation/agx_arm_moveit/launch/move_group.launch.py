"""启动 MoveIt 规划节点 move_group（路径规划 + 轨迹执行核心）。

由 demo.launch.py 包含调用，一般不单独启动。

职责：
  - 接收 RViz / 外部节点的规划请求（MoveGroup action）
  - 调用 OMPL 等规划器生成轨迹
  - 通过 arm_controller 执行轨迹（follow=false 时走 ros2_control）

关键参数：
  - follow=true  : joint_states 重映射到 feedback_topic（真机模式）
  - follow=false : joint_states 重映射到 control_topic（仿真模式）
  - debug=true   : 用 gdb 启动，便于 C++ 层调试
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

from _moveit_config_builder import build_moveit_config, declare_common_args


def _launch(context):
    moveit_config = build_moveit_config(context)
    follow = LaunchConfiguration("follow").perform(context) == "true"
    feedback_topic = LaunchConfiguration("feedback_topic").perform(context)
    control_topic = LaunchConfiguration("control_topic").perform(context)
    # follow=true 时用真机关节反馈；false 时用仿真/控制话题
    joint_states_topic = str(feedback_topic) if follow else str(control_topic)

    # move_group 节点运行参数
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": LaunchConfiguration(
            "publish_monitored_planning_scene"
        ),
        "publish_geometry_updates": LaunchConfiguration(
            "publish_monitored_planning_scene"
        ),
        "publish_state_updates": LaunchConfiguration(
            "publish_monitored_planning_scene"
        ),
        "publish_transforms_updates": LaunchConfiguration(
            "publish_monitored_planning_scene"
        ),
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    remappings = [("joint_states", joint_states_topic)]

    return [
        # 正常模式：启动 move_group
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
            remappings=remappings,
            additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
            condition=UnlessCondition(LaunchConfiguration("debug")),
        ),
        # 调试模式：用 gdb 启动 move_group
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
            remappings=remappings,
            prefix=["gdb -x {} --ex run --args".format(
                moveit_config.package_path / "launch" / "gdb_settings.gdb"
            )],
            additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
            condition=IfCondition(LaunchConfiguration("debug")),
        ),
    ]


def generate_launch_description():
    from launch.actions import DeclareLaunchArgument

    return LaunchDescription(
        declare_common_args()
        + [
            DeclareBooleanLaunchArg("debug", default_value=False),
            DeclareBooleanLaunchArg(
                "allow_trajectory_execution", default_value=True
            ),
            DeclareBooleanLaunchArg(
                "publish_monitored_planning_scene", default_value=True
            ),
            DeclareBooleanLaunchArg("monitor_dynamics", default_value=False),
            DeclareLaunchArgument("capabilities", default_value=""),
            DeclareLaunchArgument("disable_capabilities", default_value=""),
            OpaqueFunction(function=_launch),
        ]
    )
