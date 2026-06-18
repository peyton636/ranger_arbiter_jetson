"""单臂驱动 + MoveIt 运动规划一键启动。

同时启动：
  1. agx_arm_ctrl 真机驱动（CAN 通信、关节反馈、轨迹执行）
  2. agx_arm_moveit/demo.launch.py（move_group、RViz、ros2_control 等）

典型用途：在 RViz 中 Plan/Execute，轨迹经 ros2_control → control/joint_states → 真机。

启动方式：
  ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py
  ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\
      can_port:=can0 arm_type:=piper_l effector_type:=agx_gripper \\
      auto_control_gate:=true

安全门控（auto_control_gate）：
  - true  : 平时禁止 /control/*，仅在 MoveIt Execute 阶段由 control_gate 开门
  - false : 驱动节点始终接受控制指令（control_enabled=true）
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, IfElseSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def generate_launch_description():
    """组装「驱动层 + MoveIt 规划层」两个子 launch。"""

    # ── 通用参数 ──────────────────────────────────────────────────────
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for this arm instance (e.g. arm1).'
    )

    # ── 硬件与机型参数 ────────────────────────────────────────────────
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the AGX Arm node.'
    )

    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='piper',
        choices=['nero', 'piper', 'piper_h', 'piper_l', 'piper_x'],
        description='Robotic arm type (e.g. nero, piper, piper_h, piper_l, piper_x).'
    )

    effector_type_arg = DeclareLaunchArgument(
        'effector_type',
        default_value='none',
        choices=['none', 'agx_gripper', 'revo2'],
        description='End effector type (e.g. agx_gripper, revo2).'
    )

    revo2_type_arg = DeclareLaunchArgument(
       'revo2_type',
        default_value='left',
        choices=['left', 'right'],
        description='Revo2 end effector type (e.g. left, right).'
    )

    # ── 运动与安全参数 ────────────────────────────────────────────────
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        choices=['true', 'false'],
        description='Automatically enable the AGX Arm node.'
    )

    fast_mode_arg = DeclareLaunchArgument(
        'fast_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable fast mode for the AGX Arm node.'
    )

    speed_percent_arg = DeclareLaunchArgument(
        'speed_percent',
        default_value='100',
        description='Movement speed as a percentage of maximum speed.'
    )

    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200',
        description='Publishing rate for the AGX Arm node.'
    )

    enable_timeout_arg = DeclareLaunchArgument(
        'enable_timeout',
        default_value='5.0',
        description='Timeout in seconds for arm enable/disable operations.'
    )

    tcp_offset_arg = DeclareLaunchArgument(
        'tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='TCP offset in x, y, z, roll, pitch, yaw in meters/radians.'
    )

    gripper_default_effort_arg = DeclareLaunchArgument(
        'gripper_default_effort',
        default_value='1.0',
        description='Default effort for gripper commands (>= 0.0).'
    )

    # ── MoveIt 关节状态源与门控 ───────────────────────────────────────
    follow_arg = DeclareLaunchArgument(
        'follow',
        default_value='true',
        choices=['true', 'false'],
        description='Follow real arm state.',
    )#默认为true，真机模式

    feedback_topic_arg = DeclareLaunchArgument(
        'feedback_topic',
        default_value='feedback/joint_states',
        description='Joint states feedback topic for MoveIt (follow:=true).',
    )

    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='control/joint_states',
        description='Joint states control topic for MoveIt (follow:=false, ros2_control remap).',
    )

    auto_control_gate_arg = DeclareLaunchArgument(
        'auto_control_gate',
        default_value='false',
        choices=['true', 'false'],
        description='Open control gate only during MoveIt execute stage.',
    )

    control_gate_service_arg = DeclareLaunchArgument(
        'control_gate_service',
        default_value='control_enable',
        description='SetBool gate service for agx_arm_control_gate when auto_control_gate:=true.',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Optional RViz config path passed to agx_arm_moveit demo.launch.py.',
    )

    # ── 子 launch 1：真机 CAN 驱动 ───────────────────────────────────
    agx_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_ctrl'),
                'launch',
                'start_single_agx_arm.launch.py',
            )
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
            'namespace': LaunchConfiguration('namespace'),
            'can_port': LaunchConfiguration('can_port'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'arm_type': LaunchConfiguration('arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'effector_type': LaunchConfiguration('effector_type'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            # 门控开启时默认禁止控制，Execute 阶段由 agx_arm_control_gate 调用 control_enable 开门
            'control_enabled': IfElseSubstitution(
                LaunchConfiguration('auto_control_gate'),
                if_value='false',
                else_value='true',
            ),
        }.items(),
    )

    # ── 子 launch 2：MoveIt 全套（规划、RViz、ros2_control） ─────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_moveit'),
                'launch',
                'demo.launch.py',
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'revo2_type': LaunchConfiguration('revo2_type'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'follow': LaunchConfiguration('follow'),
            'use_sim': 'false',
            'feedback_topic': LaunchConfiguration('feedback_topic'),
            'control_topic': LaunchConfiguration('control_topic'),
            'auto_control_gate': LaunchConfiguration('auto_control_gate'),
            'control_gate_service': LaunchConfiguration('control_gate_service'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    return LaunchDescription([
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
        rviz_config_arg,
        agx_arm_launch,
        moveit_launch,
    ])
