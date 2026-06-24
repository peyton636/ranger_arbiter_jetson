from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rs232_gateway_share = get_package_share_directory('rs232_gateway')
    default_port = (
        '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
    )
    autostart = LaunchConfiguration('autostart')

    agv_node = LifecycleNode(
        package='agv_base_driver',
        executable='agv_base_bringe_node',
        name='agv_base_bringe',
        namespace='',
        output='screen',
        parameters=[
            {'link_type': 'rs232'},
            {'cmd_timeout_ms': LaunchConfiguration('cmd_timeout_ms')},
            {'cruise_scale': LaunchConfiguration('cruise_scale')},
        ],
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
        DeclareLaunchArgument('serial_port', default_value=default_port),
        DeclareLaunchArgument('cmd_timeout_ms', default_value='2000'),
        DeclareLaunchArgument('cruise_scale', default_value='1.0'),
        DeclareLaunchArgument('tx_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('uplink_timeout_ms', default_value='300'),
        DeclareLaunchArgument('time_sync_enable', default_value='true'),
        DeclareLaunchArgument('use_blob_v2', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rs232_gateway_share, 'launch', 'rs232_gateway.launch.py')
            ),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'tx_rate_hz': LaunchConfiguration('tx_rate_hz'),
                'uplink_timeout_ms': LaunchConfiguration('uplink_timeout_ms'),
                'time_sync_enable': LaunchConfiguration('time_sync_enable'),
                'use_blob_v2': LaunchConfiguration('use_blob_v2'),
            }.items(),
        ),
        agv_node,
        configure_handler,
        activate_handler,
    ])
