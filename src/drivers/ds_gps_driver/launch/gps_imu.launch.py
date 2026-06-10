from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gps_port = DeclareLaunchArgument(
        "gps_port",
        default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    )
    imu_port = DeclareLaunchArgument("imu_port", default_value="/dev/imu_usb")

    gps_node = Node(
        package="ds_gps_driver",
        executable="gps_serial",
        name="gps_serial_driver",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("gps_port")},
            {"baud": 9600},
            {"frame_id": "gps"},
        ],
    )
    imu_node = Node(
        package="ds_imu_driver",
        executable="wit_imu",
        name="wit_imu",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("imu_port")},
            {"baud": 9600},
            {"frame_id": "base_link"},
        ],
    )
    return LaunchDescription([gps_port, imu_port, gps_node, imu_node])
