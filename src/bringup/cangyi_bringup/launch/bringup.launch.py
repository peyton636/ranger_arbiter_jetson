import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='control',
        choices=['control', 'rviz', 'moveit'],
        description='Bringup mode: control (driver only), rviz (driver + display), moveit (driver + MoveIt).',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for this arm instance (for multi-arm isolation).',
    )
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port used by AGX arm control node.',
    )
    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='piper',
        choices=['nero', 'piper', 'piper_h', 'piper_l', 'piper_x'],
        description='Arm type.',
    )
    effector_type_arg = DeclareLaunchArgument(
        'effector_type',
        default_value='none',
        choices=['none', 'agx_gripper', 'revo2'],
        description='End effector type.',
    )
    revo2_type_arg = DeclareLaunchArgument(
        'revo2_type',
        default_value='left',
        choices=['left', 'right'],
        description='Revo2 side.',
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        choices=['true', 'false'],
        description='Auto enable AGX arm control node.',
    )
    fast_mode_arg = DeclareLaunchArgument(
        'fast_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable fast mode in control node.',
    )
    speed_percent_arg = DeclareLaunchArgument(
        'speed_percent',
        default_value='100',
        description='Max speed percentage.',
    )
    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200',
        description='Status publish rate.',
    )
    enable_timeout_arg = DeclareLaunchArgument(
        'enable_timeout',
        default_value='5.0',
        description='Enable timeout in seconds.',
    )
    tcp_offset_arg = DeclareLaunchArgument(
        'tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='TCP offset [x, y, z, roll, pitch, yaw].',
    )
    gripper_default_effort_arg = DeclareLaunchArgument(
        'gripper_default_effort',
        default_value='1.0',
        description='Default effort for gripper commands.',
    )
    follow_arg = DeclareLaunchArgument(
        'follow',
        default_value='true',
        choices=['true', 'false'],
        description='MoveIt/Display follow real arm state.',
    )
    feedback_topic_arg = DeclareLaunchArgument(
        'feedback_topic',
        default_value='feedback/joint_states',
        description='Joint state feedback topic for follow mode.',
    )
    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='control/joint_states',
        description='Joint state command topic for non-follow mode.',
    )
    auto_control_gate_arg = DeclareLaunchArgument(
        'auto_control_gate',
        default_value='false',
        choices=['true', 'false'],
        description='Enable MoveIt execution-stage control gate.',
    )
    control_gate_service_arg = DeclareLaunchArgument(
        'control_gate_service',
        default_value='control_enable',
        description='SetBool gate service name used by MoveIt control gate.',
    )

    agx_arm_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_ctrl'),
                'launch',
                'start_single_agx_arm.launch.py',
            )
        ),
        condition=IfCondition(
            PythonExpression([
                '"',
                LaunchConfiguration('mode'),
                '" == "control"',
            ])
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
            'namespace': LaunchConfiguration('namespace'),
            'can_port': LaunchConfiguration('can_port'),
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
        }.items(),
    )

    agx_arm_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_ctrl'),
                'launch',
                'start_single_agx_arm_rviz.launch.py',
            )
        ),
        condition=IfCondition(
            PythonExpression([
                '"',
                LaunchConfiguration('mode'),
                '" == "rviz"',
            ])
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
            'namespace': LaunchConfiguration('namespace'),
            'can_port': LaunchConfiguration('can_port'),
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'revo2_type': LaunchConfiguration('revo2_type'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            'follow': LaunchConfiguration('follow'),
            'feedback_topic': LaunchConfiguration('feedback_topic'),
            'control_topic': LaunchConfiguration('control_topic'),
        }.items(),
    )

    agx_arm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_ctrl'),
                'launch',
                'start_single_agx_arm_moveit.launch.py',
            )
        ),
        condition=IfCondition(
            PythonExpression([
                '"',
                LaunchConfiguration('mode'),
                '" == "moveit"',
            ])
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
            'namespace': LaunchConfiguration('namespace'),
            'can_port': LaunchConfiguration('can_port'),
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'revo2_type': LaunchConfiguration('revo2_type'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            'follow': LaunchConfiguration('follow'),
            'feedback_topic': LaunchConfiguration('feedback_topic'),
            'control_topic': LaunchConfiguration('control_topic'),
            'auto_control_gate': LaunchConfiguration('auto_control_gate'),
            'control_gate_service': LaunchConfiguration('control_gate_service'),
        }.items(),
    )

    return LaunchDescription([
        mode_arg,
        log_level_arg,
        namespace_arg,
        can_port_arg,
        arm_type_arg,
        effector_type_arg,
        revo2_type_arg,
        auto_enable_arg,
        fast_mode_arg,
        speed_percent_arg,
        pub_rate_arg,
        enable_timeout_arg,
        tcp_offset_arg,
        gripper_default_effort_arg,
        follow_arg,
        feedback_topic_arg,
        control_topic_arg,
        auto_control_gate_arg,
        control_gate_service_arg,
        agx_arm_ctrl_launch,
        agx_arm_rviz_launch,
        agx_arm_moveit_launch,
    ])
