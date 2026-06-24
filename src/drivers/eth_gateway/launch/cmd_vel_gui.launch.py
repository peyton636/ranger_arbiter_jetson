# cmd_vel GUI only (run jetson_eth_bringup separately).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("agv_control_topic", default_value="/agv_control"),
            DeclareLaunchArgument(
                "feature_status_topic", default_value="/Function/FeatureStatusInfo"
            ),
            DeclareLaunchArgument(
                "vehicle_data_topic", default_value="/Vehicle/VehicleData"
            ),
            DeclareLaunchArgument("fix_topic", default_value="/fix"),
            DeclareLaunchArgument("max_linear_m_s", default_value="0.8"),
            DeclareLaunchArgument("max_angular_rad_s", default_value="1.0"),
            DeclareLaunchArgument("publish_hz", default_value="20.0"),
            Node(
                package="eth_gateway",
                executable="cmd_vel_gui",
                name="cmd_vel_gui",
                output="screen",
                parameters=[
                    {"agv_control_topic": LaunchConfiguration("agv_control_topic")},
                    {
                        "feature_status_topic": LaunchConfiguration(
                            "feature_status_topic"
                        )
                    },
                    {"vehicle_data_topic": LaunchConfiguration("vehicle_data_topic")},
                    {"fix_topic": LaunchConfiguration("fix_topic")},
                    {"max_linear_m_s": LaunchConfiguration("max_linear_m_s")},
                    {"max_angular_rad_s": LaunchConfiguration("max_angular_rad_s")},
                    {"publish_hz": LaunchConfiguration("publish_hz")},
                ],
            ),
        ]
    )
