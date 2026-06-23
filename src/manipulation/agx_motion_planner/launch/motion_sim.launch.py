"""
仿真运动规划启动文件
只启动：
agx_motion_planner_node
agx_arm_controller_node
agx_gripper_controller_node

不启动
demo.launch.py
MoveIt move_group
RViz
robot_state_publisher
ros2_control仿真地盘/机械臂描述

使用前需要你先启动moveit/robot_description/joint_states/TF
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #1.参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='是否自动 configure/activate motion planner lifecycle node'
    )

    start_arm_controller_arg = DeclareLaunchArgument(
        'start_arm_controller',
        default_value='true',
        description='是否启动机械臂控制节点'
    )

    start_gripper_controller_arg = DeclareLaunchArgument(
        'start_gripper_controller',
        default_value='true',
        description='是否启动夹爪控制节点'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志等级'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('agx_motion_planner'),
            'config',
            'motion_planner_params.yaml',
        ]),
        description='motion planner 参数文件'
    )

    #2.启动motion planner 这里启动的是cmakelists.txt 注册出来的executable, agx_motion_planner_node
    motion_planner_node = Node(
        package='agx_motion_planner',
        executable='agx_motion_planner_node',
        name='agx_motion_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            },
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level'),
        ],
    )

    #3.启动arm controller 仿真follow=false
    arm_controller_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('agx_arm_controller'),
                        'launch',
                        'arm_controller_node.launch.py',
                    ])
                ),
                launch_arguments={
                    'follow':'false',
                }.items(),
            )
        ],
        condition=IfCondition(LaunchConfiguration('start_arm_controller')),
    )

    #4.启动gripper controller
    gripper_controller_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('agx_gripper_controller'),
                        'launch',
                        'gripper_controller_node.launch.py',
                    ])
                )
            )
        ],
        condition=IfCondition(LaunchConfiguration('start_gripper_controller')),
    )

    #5.自动激活lifecycle
    configure_motion_planner = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'lifecycle',
                    'set',
                    '/agx_motion_planner_node',
                    'configure',
                ],
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('autostart')),
    )

    activate_motion_planner = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'lifecycle',
                    'set',
                    '/agx_motion_planner_node',
                    'activate',
                ],
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('autostart')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        start_arm_controller_arg,
        start_gripper_controller_arg,
        log_level_arg,
        params_file_arg,

        motion_planner_node,
        arm_controller_launch,
        gripper_controller_launch,
        configure_motion_planner,
        activate_motion_planner,
    ])



