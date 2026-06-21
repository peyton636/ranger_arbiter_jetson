"""启动 motion_planner_node（需先启动 MoveIt move_group）。"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    pkg_share = get_package_share_directory('agx_motion_planner')
    moveit_pkg = get_package_share_directory('agx_arm_moveit')
    params_file = os.path.join(pkg_share, 'config', 'motion_planner_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to the motion planner parameters YAML file'
    )

    arm_type_arg = DeclareLaunchArgument(
        'arm_type', 
        default_value='piper_l',
        description='Type of the robotic arm (e.g., piper_l, piper_r, ur5e)'
    )

    effector_type_arg = DeclareLaunchArgument(
        'effector_type', 
        default_value='agx_gripper',
        description='Type of the end effector (e.g., agx_gripper, robotiq_2f_85)'
    )

    follow_arg = DeclareLaunchArgument(
        'follow',
        default_value='false',
        description='Whether to follow the target'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='auto',
        choices=['auto', 'true', 'false'],
        description='Whether to use simulation mode'
    )
    
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='agx_motion_planner_container',
        description='Name of the composable node container'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'demo.launch.py')),
        launch_arguments={
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'follow': LaunchConfiguration('follow'),
            'use_sim': LaunchConfiguration('use_sim'),
        }.items(),
    )


    # Composable node container (multi-threaded)
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='agx_motion_planner',
                plugin='manipulation::MotionPlannerNode',
                name='agx_motion_planner_node',
                parameters=[
                    LaunchConfiguration('params_file'),{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ]
            )
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        arm_type_arg,
        effector_type_arg,
        follow_arg,
        use_sim_arg,
        params_file_arg,
        moveit_launch,
        container_name_arg,
        use_sim_time_arg,
        log_level_arg,
        container,
    ])
