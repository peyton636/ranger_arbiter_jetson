"""启动 arm_controller_node。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agx_arm_controller')
    params_file = os.path.join(pkg_share, 'config', 'arm_controller_params.yaml')

    follow_arg = DeclareLaunchArgument(
        'follow',
        default_value='false',
        description='true: 真机反馈 /feedback/joint_states；false: 仿真反馈 /control/joint_states',
    )

    feedback_topic = PythonExpression([
        "'/feedback/joint_states' if '",
        LaunchConfiguration('follow'),
        "' == 'true' else '/control/joint_states'",
    ])
    use_ros2_control_action = PythonExpression([
        "'true' if '",
        LaunchConfiguration('follow'),
        "' == 'false' else 'false'",
    ])

    return LaunchDescription([
        follow_arg,
        Node(
            package='agx_arm_controller',
            executable='arm_controller_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'feedback_joint_states_topic': feedback_topic,
                    'use_ros2_control_action': use_ros2_control_action,
                },
            ],
        ),
    ])
