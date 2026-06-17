from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_port = (
        "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value=default_port),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("tx_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("uplink_timeout_ms", default_value="300"),
            DeclareLaunchArgument("auto_reconnect", default_value="true"),
            DeclareLaunchArgument("publish_raw", default_value="false"),
            DeclareLaunchArgument("heartbeat_mode_req", default_value="1"),
            DeclareLaunchArgument("time_sync_enable", default_value="true"),
            DeclareLaunchArgument("time_sync_ping_interval_s", default_value="1.0"),
            DeclareLaunchArgument("time_sync_query_interval_s", default_value="10.0"),
            DeclareLaunchArgument("time_sync_rtt_warn_ms", default_value="50.0"),
            DeclareLaunchArgument("use_blob_v2", default_value="true"),
            DeclareLaunchArgument("debug_rx_stats_interval_s", default_value="5.0"),
            Node(
                package="rs232_gateway",
                executable="rs232_gateway",
                name="rs232_gateway",
                output="screen",
                parameters=[
                    {"serial_port": LaunchConfiguration("serial_port")},
                    {"baud_rate": LaunchConfiguration("baud_rate")},
                    {"tx_rate_hz": LaunchConfiguration("tx_rate_hz")},
                    {"uplink_timeout_ms": LaunchConfiguration("uplink_timeout_ms")},
                    {"auto_reconnect": LaunchConfiguration("auto_reconnect")},
                    {"publish_raw": LaunchConfiguration("publish_raw")},
                    {"heartbeat_mode_req": LaunchConfiguration("heartbeat_mode_req")},
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
                    {
                        "time_sync_rtt_warn_ms": LaunchConfiguration(
                            "time_sync_rtt_warn_ms"
                        )
                    },
                    {"use_blob_v2": LaunchConfiguration("use_blob_v2")},
                    {
                        "debug_rx_stats_interval_s": LaunchConfiguration(
                            "debug_rx_stats_interval_s"
                        )
                    },
                ],
            ),
        ]
    )
