from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')

    agv_node = LifecycleNode(
        package='agv_base_driver',
        executable='agv_base_bringe_node',
        name='agv_base_bringe',
        namespace='',
        output='screen',
        parameters=[{
            'link_type': LaunchConfiguration('link_type'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'cmd_rate_hz': LaunchConfiguration('cmd_rate_hz'),
            'cmd_timeout_ms': LaunchConfiguration('cmd_timeout_ms'),
            'cruise_scale': LaunchConfiguration('cruise_scale'),
            'max_linear_m_s': LaunchConfiguration('max_linear_m_s'),
            'max_angular_rad_s': LaunchConfiguration('max_angular_rad_s'),
        }],
    )

    configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=agv_node,
            on_start=[
                LogInfo(msg='Configuring agv_base_bringe'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(agv_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=agv_node,
            goal_state='inactive',
            entities=[
                LogInfo(msg='Activating agv_base_bringe'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(agv_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument('link_type', default_value='rs232'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('cmd_timeout_ms', default_value='500'),
        DeclareLaunchArgument('cruise_scale', default_value='1.0'),
        DeclareLaunchArgument('max_linear_m_s', default_value='0.8'),
        DeclareLaunchArgument('max_angular_rad_s', default_value='1.0'),
        DeclareLaunchArgument('autostart', default_value='true'),
        agv_node,
        configure_handler,
        activate_handler,
    ])
