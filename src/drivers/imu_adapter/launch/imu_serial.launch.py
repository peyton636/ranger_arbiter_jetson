from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle.matchers import matches_action
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')

    imu_node = LifecycleNode(
        package='imu_adapter',
        executable='imu_adapter_node',
        name='imu_adapter',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id'),
            'imu_topic': 'imu/data',
            'mag_topic': 'imu/mag',
            'link_type': LaunchConfiguration('link_type'),
            'use_time_sync_stamp': LaunchConfiguration('use_time_sync_stamp'),
        }],
    )

    configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=imu_node,
            on_start=[
                LogInfo(msg='Configuring imu_adapter'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=imu_node,
            goal_state='inactive',
            entities=[
                LogInfo(msg='Activating imu_adapter'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/imu_usb'),
        DeclareLaunchArgument('baud', default_value='9600'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('link_type', default_value='eth'),
        DeclareLaunchArgument('use_time_sync_stamp', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        imu_node,
        configure_handler,
        activate_handler,
    ])
