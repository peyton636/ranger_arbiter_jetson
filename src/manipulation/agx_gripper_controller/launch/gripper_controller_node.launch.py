"""启动 gripper_controller_node。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agx_gripper_controller')
    params_file = os.path.join(pkg_share, 'config', 'gripper_controller_params.yaml')

    return LaunchDescription([
        Node(
            package='agx_gripper_controller',
            executable='gripper_controller_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
