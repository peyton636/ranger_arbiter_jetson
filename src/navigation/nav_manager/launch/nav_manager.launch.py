"""Launch nav_manager as a composable lifecycle node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_manager')
    default_params = os.path.join(pkg_share, 'config', 'nav_manager_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        ComposableNodeContainer(
            name='nav_manager_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='nav_manager',
                    plugin='navigation::NavManagerNode',
                    name='nav_manager_node',
                    parameters=[
                        LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    ],
                )
            ],
            output='screen',
        ),
    ])
