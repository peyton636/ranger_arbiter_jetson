from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_port", default_value="/dev/ttyUSB5"),
            DeclareLaunchArgument("cmd_baud", default_value="115200"),
            DeclareLaunchArgument("rate_hz", default_value="50.0"),
            DeclareLaunchArgument("cmd_timeout_ms", default_value="500"),
            DeclareLaunchArgument("mode_req", default_value="1"),
            DeclareLaunchArgument("cruise_scale", default_value="1.0"),
            DeclareLaunchArgument("max_linear_m_s", default_value="0.8"),
            DeclareLaunchArgument("max_angular_rad_s", default_value="1.0"),
            DeclareLaunchArgument("strafe_jl_from_angular", default_value="true"),
            DeclareLaunchArgument("strafe_speed_m_s", default_value="0.3"),
            DeclareLaunchArgument("sideways_steer_millirad", default_value="1571"),
            DeclareLaunchArgument("auto_reconnect", default_value="true"),
            DeclareLaunchArgument("reconnect_interval_s", default_value="1.0"),
            DeclareLaunchArgument("reconnect_settle_s", default_value="0.15"),
            DeclareLaunchArgument("reconnect_uplink_deadline_s", default_value="2.0"),
            Node(
                package="ds_jetson_bridge",
                executable="jetson_bridge",
                name="jetson_bridge",
                output="screen",
                parameters=[
                    {"cmd_port": LaunchConfiguration("cmd_port")},
                    {"cmd_baud": LaunchConfiguration("cmd_baud")},
                    {"rate_hz": LaunchConfiguration("rate_hz")},
                    {"mode_req": LaunchConfiguration("mode_req")},
                    {"cruise_scale": LaunchConfiguration("cruise_scale")},
                    {"cmd_timeout_ms": LaunchConfiguration("cmd_timeout_ms")},
                    {"max_linear_m_s": LaunchConfiguration("max_linear_m_s")},
                    {"max_angular_rad_s": LaunchConfiguration("max_angular_rad_s")},
                    {
                        "strafe_jl_from_angular": LaunchConfiguration(
                            "strafe_jl_from_angular"
                        )
                    },
                    {"strafe_speed_m_s": LaunchConfiguration("strafe_speed_m_s")},
                    {
                        "sideways_steer_millirad": LaunchConfiguration(
                            "sideways_steer_millirad"
                        )
                    },
                    {"auto_reconnect": LaunchConfiguration("auto_reconnect")},
                    {
                        "reconnect_interval_s": LaunchConfiguration(
                            "reconnect_interval_s"
                        )
                    },
                    {"reconnect_settle_s": LaunchConfiguration("reconnect_settle_s")},
                    {
                        "reconnect_uplink_deadline_s": LaunchConfiguration(
                            "reconnect_uplink_deadline_s"
                        )
                    },
                ],
            ),
        ]
    )
