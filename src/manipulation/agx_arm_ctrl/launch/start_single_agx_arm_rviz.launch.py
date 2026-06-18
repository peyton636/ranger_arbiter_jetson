"""单臂驱动 + RViz 可视化一键启动。

在 start_single_agx_arm.launch.py 基础上，额外启动：
  1. agx_arm_description/display.launch.py — URDF 显示、TF、RViz
  2. agx_arm_ctrl 真机驱动节点

典型用途：调试真机状态、手动拖动关节滑块（control:=true 时）、观察 TF 树。

启动方式：
  ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py
  ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \\
      can_port:=can0 arm_type:=piper_l effector_type:=agx_gripper follow:=true

关键参数：
  - follow=true  : RViz 跟随真机 feedback/joint_states（默认，推荐真机调试）
  - follow=false : RViz 使用 control/joint_states（仿真/滑块控制）
  - control=true : 启动 joint_state_publisher_gui，可拖动滑块发控制指令
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def generate_launch_description():
    """组装「描述层 + 驱动层」两个子 launch。"""

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

    # ── 硬件与机型参数（透传给 agx_arm_ctrl） ─────────────────────────
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

    # ── RViz 显示相关参数（透传给 display.launch.py） ─────────────────
    follow_arg = DeclareLaunchArgument(
        'follow',
        default_value='true',
        choices=['true', 'false'],
        description='Follow real arm state.',
    )

    feedback_topic_arg = DeclareLaunchArgument(
        'feedback_topic',
        default_value='feedback/joint_states',
        description='Feedback joint states topic for display.launch (follow:=true).',
    )

    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='control/joint_states',
        description='Control joint states topic for display.launch (follow:=false / joint_state_publisher).',
    )

    control_arg = DeclareLaunchArgument(
        'control',
        default_value='false',
        choices=['true', 'false'],
        description='Whether to publish control topics from the RViz-side joint state publisher.',
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

    # ── 子 launch 1：机器人模型 + RViz ───────────────────────────────
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_description'),
                'launch',
                'display.launch.py',
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'revo2_type': LaunchConfiguration('revo2_type'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'follow': LaunchConfiguration('follow'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'control': LaunchConfiguration('control'),
            'feedback_topic': LaunchConfiguration('feedback_topic'),
            'control_topic': LaunchConfiguration('control_topic'),
        }.items(),
    )

    # ── 子 launch 2：真机 CAN 驱动 ───────────────────────────────────
    agx_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agx_arm_ctrl'),
                'launch',
                'start_single_agx_arm.launch.py',
            )
        ),
        launch_arguments={
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
        }.items(),
    )

    return LaunchDescription([
        # 所有 launch 参数（子 launch 通过 LaunchConfiguration 读取）
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
        control_arg,
        # 先启动模型显示，再启动驱动（顺序对功能无硬性要求）
        description_launch,
        agx_arm_launch,
    ])
