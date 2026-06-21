"""启动 arm_controller_node。"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode



def generate_launch_description():
    pkg_share = get_package_share_directory('agx_arm_controller')
    default_params_file = os.path.join(pkg_share, 'config', 'arm_controller_params.yaml')

    params_file_config = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to params yaml'
    )

    follow_arg = DeclareLaunchArgument(
        'follow',
        default_value='false',
        description='true: 真机反馈 /feedback/joint_states; false: 仿真反馈 /control/joint_states',
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

    container_name_arg = DeclareLaunchArgument(
            'container_name',
            default_value='fusion_pose_container',
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
                package='agx_arm_controller',
                plugin='manipulation::ArmControllerNode',
                name='arm_controller_node',
                parameters=[
                    LaunchConfiguration('params_file'),{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'feedback_topic': feedback_topic,
                        'use_ros2_control_action': use_ros2_control_action,
                    }
                ]
            )
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        follow_arg,
        container_name_arg,
        log_level,
        use_sim_time,
        params_file_config,
        container,
        LogInfo(msg=['[arm_controller_node.launch] params_file=', LaunchConfiguration('params_file')]),
        LogInfo(msg=['[arm_controller_node.launch] feedback_topic=', feedback_topic]),
        LogInfo(msg=['[arm_controller_node.launch] use_ros2_control_action=', use_ros2_control_action]),
        LogInfo(msg=['[arm_controller_node.launch] use_sim_time=', LaunchConfiguration('use_sim_time')]),
    ])
