from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_params_file = PathJoinSubstitution([
        FindPackageShare('image_preprocess'),
        'config',
        'camera_params.yaml'
    ])

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'camera_params',
        default_value=default_params_file,
        description='Path to the camera parameters YAML file'
    )

    publish_timing_arg = DeclareLaunchArgument(
        'publish_timing',
        default_value='true',
        description='Whether to publish timing information'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='image_preprocess_container',
        description='Composable node container name'
    )

    # Composable node container (multi-threaded)
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='image_preprocess',
                plugin='perception::PreprocessNode',
                name='preprocess_node',
                parameters=[
                    LaunchConfiguration('camera_params'),
                    {
                        'publish_timing': LaunchConfiguration('publish_timing'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'input_topic': LaunchConfiguration('input_topic'),
                        'camera_params': LaunchConfiguration('camera_params'),
                        # 'roi_x': 0,
                        # 'roi_y': 0,
                        # 'roi_w': 0,
                        # 'roi_h': 0,
                        # 'model_names': [''],
                        # 'model_widths': [640],
                        # 'model_heights': [640],
                    }
                ]
            )
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        params_file_arg,
        publish_timing_arg,
        use_sim_time_arg,
        log_level_arg,
        input_topic_arg,
        container_name_arg,
        LogInfo(msg=['[preprocess.launch] camera_params=', LaunchConfiguration('camera_params')]),
        LogInfo(msg=['[preprocess.launch] input_topic=', LaunchConfiguration('input_topic')]),
        LogInfo(msg=['[preprocess.launch] publish_timing=', LaunchConfiguration('publish_timing')]),
        LogInfo(msg=['[preprocess.launch] use_sim_time=', LaunchConfiguration('use_sim_time')]),
        container,
    ])