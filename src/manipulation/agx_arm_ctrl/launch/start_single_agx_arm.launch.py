"""单臂 AGX 机械臂驱动启动文件（最底层入口）。

本文件只启动一个核心节点：agx_arm_ctrl_single，负责通过 CAN 总线与真机通信，
发布关节/末端/夹爪反馈，并接收运动控制指令。

启动方式：
  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py
  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \\
      can_port:=can0 arm_type:=piper_l effector_type:=agx_gripper

常用参数说明：
  - can_port       : CAN 接口名，需与系统 ifconfig 中一致
  - arm_type       : 机械臂型号，决定关节数与运动学
  - effector_type  : 末端类型（无/夹爪/Revo2 灵巧手）
  - auto_enable    : 启动后是否自动使能机械臂
  - control_enabled: 是否接受 /control/* 话题与服务（安全门控可设为 false）

话题与服务接口详见：agx_arm_ros/docs/ROS2_INTERFACE.md

上层封装：
  - start_single_agx_arm_rviz.launch.py   : 本文件 + RViz 可视化
  - start_single_agx_arm_moveit.launch.py   : 本文件 + MoveIt 规划
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

# 开启彩色日志输出，便于在终端区分 info/warn/error
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def generate_launch_description():
    """由 ros2 launch 调用，返回本 launch 包含的全部动作与节点。"""

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

    # TCP 相对法兰的偏移 [x, y, z, roll, pitch, yaw]，单位 m / rad
    # cjl：手眼标定用的tcp点和tcp_offset定义的点必须是同一个 tcp_pose=tcp_offset+flange
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

    # 控制门控：false 时节点忽略 /control/* 指令（配合 MoveIt control_gate 使用）
    control_enabled_arg = DeclareLaunchArgument(
        'control_enabled',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to accept /control/* commands.',
    )

    accept_joint_states_arm_control_arg = DeclareLaunchArgument(
        'accept_joint_states_arm_control',
        default_value='true',
        choices=['true', 'false'],
        description=(
            'When false, ignore arm joints on /control/joint_states '
            '(use with motion_stack sim + real arm).'
        ),
    )

    # ── 核心驱动节点 ────────────────────────────────────────────────
    agx_arm_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_ctrl_single_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
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
            'control_enabled': LaunchConfiguration('control_enabled'),
            'accept_joint_states_arm_control': LaunchConfiguration(
                'accept_joint_states_arm_control'),
        }],
        remappings=[
            # ── 反馈话题：真机状态 → 上层订阅 ──
            ('feedback/joint_states', 'feedback/joint_states'),       # 关节角
            ('feedback/tcp_pose', 'feedback/tcp_pose'),               # 末端位姿
            ('feedback/arm_status', 'feedback/arm_status'),             # 使能/错误等状态
            ('feedback/leader_joint_states', 'feedback/leader_joint_states'),  # 主从示教
            ('feedback/gripper_status', 'feedback/gripper_status'),   # 夹爪状态
            ('feedback/hand_status', 'feedback/hand_status'),           # 灵巧手状态

            # ── 控制话题：上层发布 → 真机执行 ──
            ('control/joint_states', 'control/joint_states'),           # 关节位置控制
            ('control/gripper_joint_states', 'control/gripper_joint_states'),  # 夹爪专用
            ('control/move_j', 'control/move_j'),                       # 关节空间运动
            ('control/move_p', 'control/move_p'),                       # 笛卡尔点位运动
            ('control/move_l', 'control/move_l'),                       # 直线运动
            ('control/move_c', 'control/move_c'),                       # 圆弧运动
            ('control/move_js', 'control/move_js'),                     # 关节空间轨迹
            ('control/move_mit', 'control/move_mit'),                   # MIT 阻抗控制
            ('control/hand', 'control/hand'),                           # 灵巧手控制
            ('control/hand_position_time', 'control/hand_position_time'),

            # ── 服务：使能、回零、急停等 ──
            ('enable_agx_arm', 'enable_agx_arm'),
            ('control_enable', 'control_enable'),       # SetBool，开关 control 话题接受
            ('move_home', 'move_home'),
            ('emergency_stop', 'emergency_stop'),
            ('exit_teach_mode', 'exit_teach_mode'),
        ],
    )

    return LaunchDescription([
        # 先声明参数，再启动节点（参数可被命令行覆盖）
        log_level_arg,
        namespace_arg,
        can_port_arg,
        arm_type_arg,
        effector_type_arg,
        auto_enable_arg,
        fast_mode_arg,
        speed_percent_arg,
        pub_rate_arg,
        enable_timeout_arg,
        tcp_offset_arg,
        gripper_default_effort_arg,
        control_enabled_arg,
        accept_joint_states_arm_control_arg,
        agx_arm_node,
    ])
