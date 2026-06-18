"""MoveIt 配置构建工具（agx_arm_moveit 各 launch 的公共依赖）。

本模块不直接启动节点，而是被 demo.launch.py、move_group.launch.py、rsp.launch.py
等子 launch 导入，根据命令行参数动态组装 MoveIt 所需的全部配置。

主要功能：
  1. declare_common_args()  — 声明各 launch 共用的参数（arm_type、follow 等）
  2. build_moveit_config()  — 读取参数，用 MoveItConfigsBuilder 加载 URDF/SRDF/控制器
  3. _select_profile()      — 按末端类型选择 moveit_controllers_*.yaml

配置选择逻辑：
  - effector_type=none        → moveit_controllers_none.yaml
  - effector_type=agx_gripper → moveit_controllers_gripper.yaml
  - effector_type=revo2       → moveit_controllers_revo2_left/right.yaml

修改指南：
  - 新增臂型：在 ALL_ARM_TYPES 添加，并确保 config/agx_arm.urdf.xacro 支持
  - 新增末端：扩展 ALL_EFFECTOR_TYPES、_select_profile()，并添加对应 yaml
  - 修改默认 TCP：调整 declare_common_args() 中 tcp_offset 的 default_value
"""

import ast

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

# 支持的机械臂型号（与 agx_arm_description 中 URDF 目录对应）
ALL_ARM_TYPES = ["piper", "piper_x", "piper_l", "piper_h", "nero"]
# 支持的末端执行器类型
ALL_EFFECTOR_TYPES = ["none", "agx_gripper", "revo2"]
# Revo2 灵巧手左右手（仅 effector_type=revo2 时生效）
ALL_REVO2_TYPES = ["left", "right"]


def declare_common_args():
    """返回各 MoveIt 子 launch 共用的 DeclareLaunchArgument 列表。

    子 launch 通过 declare_common_args() + 自身特有参数 的方式复用，
    避免在 move_group、rsp、moveit_rviz 等文件中重复声明相同参数。
    """
    return [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for this arm instance (e.g. arm1).",
        ),
        DeclareLaunchArgument(
            "arm_type", default_value="piper",
            choices=ALL_ARM_TYPES, description="Arm type.",
        ),
        DeclareLaunchArgument(
            "effector_type", default_value="none",
            choices=ALL_EFFECTOR_TYPES, description="Effector type.",
        ),
        DeclareLaunchArgument(
            "revo2_type", default_value="left",
            choices=ALL_REVO2_TYPES,
            description="Revo2 side (used when effector_type is revo2).",
        ),
        DeclareLaunchArgument(
            "tcp_offset",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            description="TCP offset [x, y, z, rx, ry, rz] in meters/radians.",
        ),
        DeclareLaunchArgument(
            "follow",
            default_value="false",
            choices=["true", "false"],
            description="Follow real arm state. "
            "true: move_group subscribes to feedback_topic; "
            "false: subscribes to control_topic (mock hardware).",
        ),
        DeclareLaunchArgument(
            "feedback_topic",
            default_value="feedback/joint_states",
            description="Joint states feedback topic (used when follow:=true).",
        ),
        DeclareLaunchArgument(
            "control_topic",
            default_value="control/joint_states",
            description="Joint states control topic (used when follow:=false, and for ros2_control_node).",
        ),
    ]


def _select_profile(effector_type: str, revo2_type: str) -> str:
    """根据末端类型选择 MoveIt 控制器配置文件名后缀。

    返回值用于拼接路径：config/moveit_controllers_{profile}.yaml
    """
    if effector_type == "agx_gripper":
        return "gripper"
    if effector_type == "revo2":
        return f"revo2_{revo2_type}"
    return "none"


def build_moveit_config(context):
    """在 OpaqueFunction 回调中调用，根据 launch 上下文构建 MoveItConfigs 对象。

    Args:
        context: launch 运行时上下文，用于 perform() 读取命令行参数。

    Returns:
        MoveItConfigs 对象，包含 robot_description、semantic、kinematics 等，
        可直接传给 move_group / robot_state_publisher / rviz2 节点。
    """
    arm_type = LaunchConfiguration("arm_type").perform(context)
    effector_type = LaunchConfiguration("effector_type").perform(context)
    revo2_type = LaunchConfiguration("revo2_type").perform(context)
    tcp_offset = ast.literal_eval(
        LaunchConfiguration("tcp_offset").perform(context)
    )

    profile = _select_profile(effector_type, revo2_type)

    # 传给 URDF xacro 的变量（臂型、末端、TCP 偏移）
    urdf_mappings = {
        "arm_type": arm_type,
        "effector_type": effector_type,
        "revo2_type": revo2_type,
        "tcp_offset_xyz": f"{tcp_offset[0]} {tcp_offset[1]} {tcp_offset[2]}",
        "tcp_offset_rpy": f"{tcp_offset[3]} {tcp_offset[4]} {tcp_offset[5]}",
    }
    # 传给 SRDF xacro 的变量（规划组、预设姿态、碰撞对等）
    srdf_mappings = {
        "arm_type": arm_type,
        "effector_type": effector_type,
        "revo2_type": revo2_type,
    }

    moveit_config = (
        MoveItConfigsBuilder("agx_arm", package_name="agx_arm_moveit")
        .robot_description(file_path="config/agx_arm.urdf.xacro", mappings=urdf_mappings)
        .robot_description_semantic(
            file_path="config/agx_arm.srdf.xacro", mappings=srdf_mappings
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .trajectory_execution(file_path=f"config/moveit_controllers_{profile}.yaml")
        .to_moveit_configs()
    )

    # Nero 为 7 轴臂，默认控制器配置只有 6 关节，需运行时覆盖关节列表
    if arm_type == "nero":
        moveit_config.trajectory_execution[
            "moveit_simple_controller_manager"
        ]["arm_controller"]["joints"] = [
            "joint1", "joint2", "joint3", "joint4",
            "joint5", "joint6", "joint7",
        ]

    return moveit_config
