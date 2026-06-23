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
from launch.actions import DeclareLaunchArgument, ExecuteProcess,EmitEvent,RegisterEventHandler, IncludeLaunchDescription,TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.msg import Transition

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

    #注意这里不要叫params_file, 避免木然arm/gripper子launch
    motion_planner_params_file_arg = DeclareLaunchArgument(
        'motion_planner_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('agx_motion_planner'),
            'config',
            'motion_planner_params.yaml',
        ]),
        description='motion planner 参数文件'
    )

    arm_controller_params_file_arg = DeclareLaunchArgument(
        'arm_controller_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('agx_arm_controller'),
            'config',
            'arm_controller_params.yaml',
        ]),
        description='arm controller 参数文件'
    )

    gripper_controller_params_file_arg = DeclareLaunchArgument(
        'gripper_controller_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('agx_gripper_controller'),
            'config',
            'gripper_controller_params.yaml',
        ]),
        description='gripper controller 参数文件'
    )

    #2.启动motion planner 这里启动的是cmakelists.txt 注册出来的executable, agx_motion_planner_node
    motion_planner_node = LifecycleNode(
        package='agx_motion_planner',
        executable='agx_motion_planner_node',
        name='agx_motion_planner_node',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('motion_planner_params_file'),
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

    #3.启动arm controller 仿真follow=false 显示传containner_name,避免和girpper或其他容器重名
    arm_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agx_arm_controller'),
                'launch',
                'agx_arm_controller_node.launch.py',
            ])
        ),
        launch_arguments={
            'follow':'false',
            'params_file':LaunchConfiguration('arm_controller_params_file'),
            'container_name':'arm_controller_container',
            'use_sim_time':LaunchConfiguration('use_sim_time'),
            'log_level':LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_arm_controller')),
    )


    #4.启动gripper controller,显示传
    gripper_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agx_gripper_controller'),
                'launch',
                'agx_gripper_controller_node.launch.py',
            ])
        ),
        launch_arguments={
            'params_file':LaunchConfiguration('gripper_controller_params_file'),
            'container_name': 'gripper_controller_container',
            'use_sim_time':LaunchConfiguration('use_sim_time'),
            'log_level':LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_gripper_controller')),
    )

    #5.自动激活lifecycle
    configure_motion_planner = RegisterEventHandler(
        OnProcessStart(
            target_action=motion_planner_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(motion_planner_node),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ],
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration('autostart')),
    )

    activate_motion_planner = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=motion_planner_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(motion_planner_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration('autostart')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        start_arm_controller_arg,
        start_gripper_controller_arg,
        log_level_arg,
        motion_planner_params_file_arg,
        arm_controller_params_file_arg,
        gripper_controller_params_file_arg,

        motion_planner_node,

        TimerAction(period=1.0, actions=[arm_controller_launch]),
        TimerAction(period=1.0, actions=[gripper_controller_launch]),

        configure_motion_planner,
        activate_motion_planner,
    ])



