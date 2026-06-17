from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("link_type", default_value="rs232"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("cmd_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("cmd_timeout_ms", default_value="500"),
            DeclareLaunchArgument("cruise_scale", default_value="1.0"),
            DeclareLaunchArgument("max_linear_m_s", default_value="0.8"),
            DeclareLaunchArgument("max_angular_rad_s", default_value="1.0"),
            Node(
                package="agv_base_driver",
                executable="agv_base_driver",
                name="agv_base_driver",
                output="screen",
                parameters=[
                    {"link_type": LaunchConfiguration("link_type")},
                    {"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")},
                    {"cmd_rate_hz": LaunchConfiguration("cmd_rate_hz")},
                    {"cmd_timeout_ms": LaunchConfiguration("cmd_timeout_ms")},
                    {"cruise_scale": LaunchConfiguration("cruise_scale")},
                    {"max_linear_m_s": LaunchConfiguration("max_linear_m_s")},
                    {"max_angular_rad_s": LaunchConfiguration("max_angular_rad_s")},
                ],
            ),
        ]
    )
