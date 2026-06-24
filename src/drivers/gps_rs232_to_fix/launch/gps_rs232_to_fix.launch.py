from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("link_type", default_value="eth"),
            DeclareLaunchArgument("fix_topic", default_value="/fix"),
            DeclareLaunchArgument("frame_id", default_value="gps"),
            DeclareLaunchArgument("use_time_sync_stamp", default_value="true"),
            Node(
                package="gps_rs232_to_fix",
                executable="gps_to_fix",
                name="gps_to_fix",
                output="screen",
                parameters=[
                    {"link_type": LaunchConfiguration("link_type")},
                    {"fix_topic": LaunchConfiguration("fix_topic")},
                    {"frame_id": LaunchConfiguration("frame_id")},
                    {
                        "use_time_sync_stamp": LaunchConfiguration(
                            "use_time_sync_stamp"
                        )
                    },
                ],
            ),
        ]
    )
