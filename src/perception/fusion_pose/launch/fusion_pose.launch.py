import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('fusion_pose')
    default_params_file = os.path.join(pkg_share, 'config', 'hand_eye_result.yaml')

    params_file_config = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to params yaml'
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
    input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/perception/detect/detections_2d',
        description='Input topic for image preprocess node'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='fusion_pose_container',
        description='Name of the composable node container'
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='fusion_pose',
                plugin='perception::FusionPoseNode',
                name='fusion_pose_node',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'input_topic': LaunchConfiguration('input_topic'),
                    }
                ]
            )
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        params_file_config,
        log_level,
        use_sim_time,
        input_topic,
        container_name_arg,
        container,
        LogInfo(msg=['[fusion_pose.launch] params_file=', LaunchConfiguration('params_file')]),
        LogInfo(msg=['[fusion_pose.launch] input_topic=', LaunchConfiguration('input_topic')]),
        LogInfo(msg=['[fusion_pose.launch] use_sim_time=', LaunchConfiguration('use_sim_time')]),
    ])