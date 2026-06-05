from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    interface = LaunchConfiguration("interface")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "interface",
                default_value="can2",
                description="SocketCAN interface name",
            ),
            ExecuteProcess(
                cmd=[
                    "can_monitor",
                    "-i",
                    interface,
                ],
                output="screen",
            ),
        ]
    )
