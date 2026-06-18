"""启动 motion_planner_node（需先启动 MoveIt move_group）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agx_motion_planner')
    moveit_pkg = get_package_share_directory('agx_arm_moveit')
    params_file = os.path.join(pkg_share, 'config', 'motion_planner_params.yaml')

    arm_type_arg = DeclareLaunchArgument('arm_type', default_value='piper_l')
    effector_type_arg = DeclareLaunchArgument('effector_type', default_value='agx_gripper')
    follow_arg = DeclareLaunchArgument('follow', default_value='false')
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='auto',
        choices=['auto', 'true', 'false'],
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'demo.launch.py')),
        launch_arguments={
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'follow': LaunchConfiguration('follow'),
            'use_sim': LaunchConfiguration('use_sim'),
        }.items(),
    )

    planner_node = Node(
        package='agx_motion_planner',
        executable='motion_planner_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        arm_type_arg,
        effector_type_arg,
        follow_arg,
        use_sim_arg,
        moveit_launch,
        planner_node,
    ])
