"""启动 gripper_controller_node。"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():    
    pkg_share = get_package_share_directory('agx_gripper_controller')
    default_params_file = os.path.join(pkg_share, 'config', 'gripper_controller_params.yaml')

    params_file_config = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to params yaml'
    )


    container_name_arg = DeclareLaunchArgument(
            'container_name',
            default_value='gripper_controller_container',
            description='Name of the composable node container'
    )
    log_level = DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
    )
    use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
    )

    # Composable node container (multi-threaded)
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='agx_gripper_controller',
                plugin='manipulation::GripperControllerNode',
                name='gripper_controller_node',
                parameters=[
                    LaunchConfiguration('params_file'),{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ]
            )
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )


    return LaunchDescription([
        container_name_arg,
        log_level,
        use_sim_time,
        params_file_config,
        container,
        LogInfo(msg=['[gripper_controller_node.launch] params_file=', LaunchConfiguration('params_file')]),
    ])  