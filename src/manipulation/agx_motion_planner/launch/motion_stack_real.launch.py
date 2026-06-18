"""真机运动栈一键启动（无 ros2_control 仿真）。

等价于：
  ros2 launch agx_motion_planner motion_stack.launch.py \\
      follow:=true use_sim:=false arm_type:=... effector_type:=...

需另开终端先启动 agx_arm_ctrl。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument('arm_type', default_value='piper_l')
    effector_type_arg = DeclareLaunchArgument('effector_type', default_value='agx_gripper')

    motion_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agx_motion_planner'),
                'launch',
                'motion_stack.launch.py',
            ]),
        ]),
        launch_arguments={
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'follow': 'true',
            'use_sim': 'false',
        }.items(),
    )

    return LaunchDescription([
        arm_type_arg,
        effector_type_arg,
        motion_stack_launch,
    ])
