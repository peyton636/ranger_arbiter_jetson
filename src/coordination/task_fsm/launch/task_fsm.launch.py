from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('task_fsm')
    params_file = os.path.join(pkg_share, 'config', 'task_fsm_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Task FSM parameter file',
    )

    container = ComposableNodeContainer(
        name='task_fsm_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='task_fsm_node',
                plugin='coordination::TaskFsmNode',
                name='task_fsm_node',
                parameters=[LaunchConfiguration('params_file')],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        params_arg,
        container,
    ])
