from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('object_localization'),
        'config',
        'hand_eye_result.yaml'
    ])

    params_file = DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
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
            # default_value='',
            description='Input topic for image preprocess node'
    )

    publish_timing = DeclareLaunchArgument(
            'publish_timing',
            default_value='false',
            description='Whether to publish timing information' 
    )
    container_name_arg = DeclareLaunchArgument(
            'container_name',
            default_value='object_localization_container',
            description='Name of the composable node container'
    )

    # Composable node container (multi-threaded)
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='object_localization',
                plugin='perception::ObjLocalizationNode',
                name='obj_localization_node',
                parameters=[
                    LaunchConfiguration('params_file'),{
                        'publish_timing': LaunchConfiguration('publish_timing'),
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
        params_file,
        log_level,
        use_sim_time,
        input_topic,
        publish_timing, 
        container,
        container_name_arg,
        LogInfo(msg=['[preprocess.launch] params_file=', LaunchConfiguration('params_file')]),
        LogInfo(msg=['[preprocess.launch] input_topic=', LaunchConfiguration('input_topic')]),
        LogInfo(msg=['[preprocess.launch] publish_timing=', LaunchConfiguration('publish_timing')]),
        LogInfo(msg=['[preprocess.launch] use_sim_time=', LaunchConfiguration('use_sim_time')]),
    ])