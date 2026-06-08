from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/imu_usb"),
            DeclareLaunchArgument("baud", default_value="9600"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            Node(
                package="ds_imu_driver",
                executable="wit_imu",
                name="wit_imu",
                output="screen",
                parameters=[
                    {"port": LaunchConfiguration("port")},
                    {"baud": LaunchConfiguration("baud")},
                    {"frame_id": LaunchConfiguration("frame_id")},
                    {"imu_topic": "imu/data"},
                    {"mag_topic": "imu/mag"},
                ],
            ),
        ]
    )
