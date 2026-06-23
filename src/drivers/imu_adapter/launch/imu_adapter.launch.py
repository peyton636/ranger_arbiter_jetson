import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle.matchers import matches_action
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = get_package_share_directory('imu_adapter')
    default_params = os.path.join(pkg_share, 'config', 'imu_adapter_params.yaml')

    params_file = DeclareLaunchArgument('params_file', default_value=default_params)
    log_level = DeclareLaunchArgument('log_level', default_value='info')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    use_composition = DeclareLaunchArgument('use_composition', default_value='true')
    autostart = DeclareLaunchArgument('autostart', default_value='true')
    container_name = DeclareLaunchArgument('container_name', default_value='imu_adapter_container')

    composable_node = ComposableNode(
        package='imu_adapter',
        plugin='imu_adapter::ImuAdapterNode',
        name='imu_adapter',
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

    standalone_node = LifecycleNode(
        package='imu_adapter',
        executable='imu_adapter_node',
        name='imu_adapter',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=UnlessCondition(LaunchConfiguration('use_composition')),
    )

    configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=standalone_node,
            on_start=[
                LogInfo(msg='Configuring imu_adapter'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(standalone_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=standalone_node,
            goal_state='inactive',
            entities=[
                LogInfo(msg='Activating imu_adapter'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(standalone_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        params_file,
        log_level,
        use_sim_time,
        use_composition,
        autostart,
        container_name,
        container,
        standalone_node,
        configure_handler,
        activate_handler,
        LogInfo(msg=['[imu_adapter.launch] params_file=', LaunchConfiguration('params_file')]),
    ])
