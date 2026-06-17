from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/usb-Prolific_Technology_Inc._"
                    "USB-Serial_Controller-if00-port0"
                ),
            ),
            DeclareLaunchArgument("imu_port", default_value="/dev/imu_usb"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("agv_base_driver"),
                        "launch",
                        "jetson_rs232_bringup.launch.py",
                    ])
                ),
                launch_arguments={
                    "serial_port": LaunchConfiguration("serial_port"),
                }.items(),
            ),
            Node(
                package="gps_rs232_to_fix",
                executable="gps_rs232_to_fix",
                name="gps_rs232_to_fix",
                output="screen",
                parameters=[{"link_type": "rs232"}],
            ),
            Node(
                package="ds_imu_driver",
                executable="wit_imu",
                name="wit_imu",
                output="screen",
                parameters=[
                    {"port": LaunchConfiguration("imu_port")},
                    {"imu_topic": "imu/data"},
                ],
            ),
            Node(
                package="ds_imu_gps_localization",
                executable="imu_gps_localization",
                name="imu_gps_localization",
                output="screen",
            ),
        ]
    )
