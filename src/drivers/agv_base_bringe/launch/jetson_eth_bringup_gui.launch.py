# Ethernet bringup + cmd_vel GUI (single launch).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("agv_base_driver")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_ip", default_value="192.168.10.201"),
            DeclareLaunchArgument("mcu_ip", default_value="192.168.10.30"),
            DeclareLaunchArgument("max_linear_m_s", default_value="0.8"),
            DeclareLaunchArgument("max_angular_rad_s", default_value="1.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "jetson_eth_bringup.launch.py")
                ),
                launch_arguments={
                    "bind_ip": LaunchConfiguration("bind_ip"),
                    "mcu_ip": LaunchConfiguration("mcu_ip"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "cmd_vel_gui.launch.py")
                ),
                launch_arguments={
                    "status_topic": "/jetson_eth/v3_status",
                    "max_linear_m_s": LaunchConfiguration("max_linear_m_s"),
                    "max_angular_rad_s": LaunchConfiguration("max_angular_rad_s"),
                }.items(),
            ),
        ]
    )
