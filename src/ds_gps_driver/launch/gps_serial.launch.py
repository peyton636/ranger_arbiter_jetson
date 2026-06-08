from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            DeclareLaunchArgument("baud", default_value="9600"),
            DeclareLaunchArgument("frame_id", default_value="gps"),
            Node(
                package="ds_gps_driver",
                executable="gps_serial",
                name="gps_serial_driver",
                output="screen",
                parameters=[
                    {"port": LaunchConfiguration("port")},
                    {"baud": LaunchConfiguration("baud")},
                    {"frame_id": LaunchConfiguration("frame_id")},
                    {"fix_topic": "fix"},
                    {"vel_topic": "vel"},
                ],
            ),
        ]
    )
