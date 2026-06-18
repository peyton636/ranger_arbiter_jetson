"""一键启动规划决策层三节点（motion_planner + arm_controller + gripper_controller）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument('arm_type', default_value='piper_l')
    effector_type_arg = DeclareLaunchArgument('effector_type', default_value='agx_gripper')
    follow_arg = DeclareLaunchArgument('follow', default_value='false')
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='auto',
        choices=['auto', 'true', 'false'],
        description=(
            'Start ros2_control sim stack. auto: off when follow:=true. '
            'Real hardware: follow:=true use_sim:=false.'
        ),
    )

    motion_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agx_motion_planner'),
                'launch',
                'motion_planner_node.launch.py',
            ]),
        ]),
        launch_arguments={
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'follow': LaunchConfiguration('follow'),
            'use_sim': LaunchConfiguration('use_sim'),
        }.items(),
    )

    arm_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agx_arm_controller'),
                'launch',
                'arm_controller_node.launch.py',
            ]),
        ]),
        launch_arguments={
            'follow': LaunchConfiguration('follow'),
        }.items(),
    )

    gripper_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agx_gripper_controller'),
                'launch',
                'gripper_controller_node.launch.py',
            ]),
        ]),
    )

    return LaunchDescription([
        arm_type_arg,
        effector_type_arg,
        follow_arg,
        use_sim_arg,
        motion_planner_launch,
        TimerAction(period=2.0, actions=[arm_controller_launch]),
        TimerAction(period=2.0, actions=[gripper_controller_launch]),
    ])
