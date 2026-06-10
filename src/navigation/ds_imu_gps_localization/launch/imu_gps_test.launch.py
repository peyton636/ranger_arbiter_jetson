from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gps_port = DeclareLaunchArgument(
        "gps_port",
        default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    )
    imu_port = DeclareLaunchArgument("imu_port", default_value="/dev/imu_usb")

    gps_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ds_gps_driver"),
                "launch",
                "gps_imu.launch.py",
            ])
        ]),
        launch_arguments={
            "gps_port": LaunchConfiguration("gps_port"),
            "imu_port": LaunchConfiguration("imu_port"),
        }.items(),
    )

    fusion = Node(
        package="ds_imu_gps_localization",
        executable="imu_gps_localization",
        name="imu_gps_localization",
        output="screen",
    )

    return LaunchDescription([gps_port, imu_port, gps_imu, fusion])
