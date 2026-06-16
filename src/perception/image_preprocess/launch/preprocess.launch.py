from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('camera_params'),
        'config',
        'camera_params.yaml'
    ])

    params_file = LaunchConfiguration('params_file')
    input_topic = LaunchConfiguration('input_topic')
    node_name = LaunchConfiguration('node_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the parameter file'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='image_preprocess_node',
            description='Name of the image preprocess node'
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/camera/color/image_raw',
            description='Input image topic'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        Node(
            package='image_preprocess',
            executable='image_preprocess_node',
            name=node_name,
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time, 'input_topic': input_topic}
            ],
            arguments=['--ros-args', '--log-level', log_level]
        )
    ])