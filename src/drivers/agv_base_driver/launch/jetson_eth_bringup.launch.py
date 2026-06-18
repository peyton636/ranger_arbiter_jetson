from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    eth_gateway_share = get_package_share_directory("eth_gateway")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_ip", default_value="192.168.10.201"),
            DeclareLaunchArgument("local_port", default_value="50002"),
            DeclareLaunchArgument("mcu_ip", default_value="192.168.10.30"),
            DeclareLaunchArgument("mcu_port", default_value="50001"),
            DeclareLaunchArgument("cmd_timeout_ms", default_value="2000"),
            DeclareLaunchArgument("cruise_scale", default_value="1.0"),
            DeclareLaunchArgument("tx_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("uplink_timeout_ms", default_value="300"),
            DeclareLaunchArgument("time_sync_enable", default_value="true"),
            DeclareLaunchArgument("time_sync_ping_interval_s", default_value="1.0"),
            DeclareLaunchArgument("time_sync_query_interval_s", default_value="10.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(eth_gateway_share, "launch", "eth_gateway.launch.py")
                ),
                launch_arguments={
                    "bind_ip": LaunchConfiguration("bind_ip"),
                    "local_port": LaunchConfiguration("local_port"),
                    "mcu_ip": LaunchConfiguration("mcu_ip"),
                    "mcu_port": LaunchConfiguration("mcu_port"),
                    "tx_rate_hz": LaunchConfiguration("tx_rate_hz"),
                    "uplink_timeout_ms": LaunchConfiguration("uplink_timeout_ms"),
                    "time_sync_enable": LaunchConfiguration("time_sync_enable"),
                    "time_sync_ping_interval_s": LaunchConfiguration(
                        "time_sync_ping_interval_s"
                    ),
                    "time_sync_query_interval_s": LaunchConfiguration(
                        "time_sync_query_interval_s"
                    ),
                }.items(),
            ),
            Node(
                package="agv_base_driver",
                executable="agv_base_driver",
                name="agv_base_driver",
                output="screen",
                parameters=[
                    {"link_type": "eth"},
                    {"cmd_timeout_ms": LaunchConfiguration("cmd_timeout_ms")},
                    {"cruise_scale": LaunchConfiguration("cruise_scale")},
                ],
            ),
        ]
    )
