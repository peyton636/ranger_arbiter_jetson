from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ds_imu_gps_localization",
            executable="imu_gps_localization",
            name="imu_gps_localization",
            output="screen",
        ),
    ])
