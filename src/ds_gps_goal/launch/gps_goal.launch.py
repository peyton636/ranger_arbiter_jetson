from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("use_nav2", default_value="true"),
        Node(
            package="ds_gps_goal",
            executable="gps_goal",
            name="gps_goal",
            output="screen",
            parameters=[
                {"frame_id": LaunchConfiguration("frame_id")},
                {"use_nav2": LaunchConfiguration("use_nav2")},
            ],
        ),
    ])
