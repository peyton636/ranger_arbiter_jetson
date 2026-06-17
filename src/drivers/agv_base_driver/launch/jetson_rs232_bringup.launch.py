from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rs232_gateway_share = get_package_share_directory("rs232_gateway")
    default_port = (
        "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value=default_port),
            DeclareLaunchArgument("cmd_timeout_ms", default_value="2000"),
            DeclareLaunchArgument("cruise_scale", default_value="1.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rs232_gateway_share, "launch", "rs232_gateway.launch.py")
                ),
                launch_arguments={
                    "serial_port": LaunchConfiguration("serial_port"),
                }.items(),
            ),
            Node(
                package="agv_base_driver",
                executable="agv_base_driver",
                name="agv_base_driver",
                output="screen",
                parameters=[
                    {"link_type": "rs232"},
                    {"cmd_timeout_ms": LaunchConfiguration("cmd_timeout_ms")},
                    {"cruise_scale": LaunchConfiguration("cruise_scale")},
                ],
            ),
        ]
    )
