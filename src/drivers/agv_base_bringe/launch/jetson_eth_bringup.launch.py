from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_ip", default_value="192.168.10.201"),
            DeclareLaunchArgument("local_port", default_value="50002"),
            DeclareLaunchArgument("mcu_ip", default_value="192.168.10.30"),
            DeclareLaunchArgument("mcu_port", default_value="50001"),
            DeclareLaunchArgument("tx_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("uplink_timeout_ms", default_value="300"),
            DeclareLaunchArgument("cmd_timeout_ms", default_value="2000"),
            DeclareLaunchArgument("cruise_scale", default_value="1.0"),
            DeclareLaunchArgument("rx_dispatch_hz", default_value="50.0"),
            DeclareLaunchArgument("time_sync_enable", default_value="true"),
            DeclareLaunchArgument("time_sync_ping_interval_s", default_value="1.0"),
            DeclareLaunchArgument("time_sync_query_interval_s", default_value="10.0"),
            DeclareLaunchArgument("gps_enable", default_value="true"),
            Node(
                package="eth_gateway",
                executable="agv_base_eth_bringe",
                name="agv_base_bringe",
                output="screen",
                parameters=[
                    {"bind_ip": LaunchConfiguration("bind_ip")},
                    {"local_port": LaunchConfiguration("local_port")},
                    {"mcu_ip": LaunchConfiguration("mcu_ip")},
                    {"mcu_port": LaunchConfiguration("mcu_port")},
                    {"tx_rate_hz": LaunchConfiguration("tx_rate_hz")},
                    {"uplink_timeout_ms": LaunchConfiguration("uplink_timeout_ms")},
                    {"rx_dispatch_hz": LaunchConfiguration("rx_dispatch_hz")},
                    {"time_sync_enable": LaunchConfiguration("time_sync_enable")},
                    {
                        "time_sync_ping_interval_s": LaunchConfiguration(
                            "time_sync_ping_interval_s"
                        )
                    },
                    {
                        "time_sync_query_interval_s": LaunchConfiguration(
                            "time_sync_query_interval_s"
                        )
                    },
                    {"cmd_timeout_ms": LaunchConfiguration("cmd_timeout_ms")},
                    {"cruise_scale": LaunchConfiguration("cruise_scale")},
                    {"gps_enable": LaunchConfiguration("gps_enable")},
                    {"ros_uplink_publish": False},
                    {"ros_link_publish": False},
                    {"ros_time_sync_publish": False},
                    {"ros_sensor_cfg_sub": False},
                ],
            ),
        ]
    )
