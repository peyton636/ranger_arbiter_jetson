from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("agv_base_driver")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_ip", default_value="192.168.10.201"),
            DeclareLaunchArgument("bind_device", default_value="enp1s0f1"),
            DeclareLaunchArgument("local_port", default_value="50002"),
            DeclareLaunchArgument("mcu_ip", default_value="192.168.10.30"),
            DeclareLaunchArgument("mcu_port", default_value="50001"),
            DeclareLaunchArgument("max_linear_m_s", default_value="0.8"),
            DeclareLaunchArgument("max_angular_rad_s", default_value="1.0"),
            DeclareLaunchArgument("cmd_timeout_ms", default_value="2000"),
            DeclareLaunchArgument("cruise_scale", default_value="1.0"),
            DeclareLaunchArgument("gps_enable", default_value="true"),
            DeclareLaunchArgument("uplink_timeout_ms", default_value="1000"),
            DeclareLaunchArgument("link_down_timeout_ms", default_value="1500"),
            Node(
                package="agv_base_driver",
                executable="agv_base_eth_bringe",
                name="agv_base_bringe",
                output="screen",
                parameters=[
                    {"bind_ip": LaunchConfiguration("bind_ip")},
                    {"bind_device": LaunchConfiguration("bind_device")},
                    {"local_port": LaunchConfiguration("local_port")},
                    {"mcu_ip": LaunchConfiguration("mcu_ip")},
                    {"mcu_port": LaunchConfiguration("mcu_port")},
                    {"cmd_timeout_ms": LaunchConfiguration("cmd_timeout_ms")},
                    {"cruise_scale": LaunchConfiguration("cruise_scale")},
                    {"gps_enable": LaunchConfiguration("gps_enable")},
                    {"uplink_timeout_ms": LaunchConfiguration("uplink_timeout_ms")},
                    {"link_down_timeout_ms": LaunchConfiguration("link_down_timeout_ms")},
                    {"ros_uplink_publish": False},
                    {"ros_link_publish": False},
                    {"ros_time_sync_publish": False},
                    {"ros_sensor_cfg_sub": False},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "cmd_vel_gui.launch.py")
                ),
                launch_arguments={
                    "max_linear_m_s": LaunchConfiguration("max_linear_m_s"),
                    "max_angular_rad_s": LaunchConfiguration("max_angular_rad_s"),
                }.items(),
            ),
        ]
    )
