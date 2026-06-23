import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_base_driver')
    default_params = os.path.join(pkg_share, 'config', 'agv_base_bringe_params.yaml')

    params_file = DeclareLaunchArgument('params_file', default_value=default_params)
    log_level = DeclareLaunchArgument('log_level', default_value='info')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    use_composition = DeclareLaunchArgument('use_composition', default_value='true')
    container_name = DeclareLaunchArgument(
        'container_name', default_value='agv_base_bringe_container')

    composable_node = ComposableNode(
        package='agv_base_driver',
        plugin='agv_base_driver::AgvBaseBringeNode',
        name='agv_base_bringe',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[composable_node],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('use_composition')),
    )

    standalone_node = Node(
        package='agv_base_driver',
        executable='agv_base_bringe_node',
        name='agv_base_bringe',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=UnlessCondition(LaunchConfiguration('use_composition')),
    )

    return LaunchDescription([
        params_file,
        log_level,
        use_sim_time,
        use_composition,
        container_name,
        container,
        standalone_node,
        LogInfo(msg=['[agv_base_bringe.launch] params_file=', LaunchConfiguration('params_file')]),
    ])
