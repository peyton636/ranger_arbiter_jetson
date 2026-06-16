from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('object_localization'),
        'config',
        'object_localization.yaml'
    ])

    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to params yaml'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        Node(
            package='object_localization',
            executable='object_localization_node',
            name='object_localization_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level]
        )
    ])