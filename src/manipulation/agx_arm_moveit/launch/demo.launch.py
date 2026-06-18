"""MoveIt 主入口：一次性启动规划、RViz、控制器等全部组件。

用法示例：
  # 纯仿真（无真机，follow=false）
  ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper_l effector_type:=agx_gripper

  # 跟随真机状态（follow=true + use_sim=false，需配合 agx_arm_ctrl 驱动）
  ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper_l follow:=true use_sim:=false

启动顺序（由 _build_moveit 组装）：
  1. static_virtual_joint_tfs  — base_link 固定到 world
  2. rsp                       — robot_state_publisher
  3. move_group                — 路径规划与轨迹执行
  4. agx_arm_control_gate       — 可选，Execute 时开门控
  5. moveit_rviz               — RViz 规划界面
  6. warehouse_db              — 可选，场景数据库
  7. ros2_control_node         — 仅 use_sim:=true 时启动（仿真）
  8. spawn_controllers         — 仅 use_sim:=true 时启动
"""

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

from _moveit_config_builder import (
    ALL_ARM_TYPES,
    ALL_EFFECTOR_TYPES,
    ALL_REVO2_TYPES,
    build_moveit_config,
)


def _build_ros2_controllers_file(arm_type, effector_type, revo2_type, namespace):
    """根据臂型和末端类型，动态生成 ros2_control 控制器配置（写入临时 yaml）。

    返回临时文件路径，供 ros2_control_node 加载。
    """
    arm_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    if arm_type == "nero":
        arm_joints.append("joint7")

    cm_controllers = {
        "arm_controller": {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        },
        "joint_state_broadcaster": {
            "type": "joint_state_broadcaster/JointStateBroadcaster",
        },
    }

    ns = namespace.strip("/")
    cm_node = f"/{ns}/controller_manager" if ns else "/controller_manager"

    config = {
        cm_node: {
            "ros__parameters": {"update_rate": 200, **cm_controllers},
        },
        (f"/{ns}/arm_controller" if ns else "/arm_controller"): {
            "ros__parameters": {
                "joints": arm_joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        },
    }

    # 带夹爪时增加 gripper_controller
    if effector_type == "agx_gripper":
        cm_controllers["gripper_controller"] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        }
        config[cm_node]["ros__parameters"].update(cm_controllers)
        config[(f"/{ns}/gripper_controller" if ns else "/gripper_controller")] = {
            "ros__parameters": {
                "joints": ["gripper"],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        }
    # 带 Revo2 灵巧手时增加对应 hand_controller
    elif effector_type == "revo2":
        side = revo2_type
        ctrl_name = f"{side}_hand_controller"
        cm_controllers[ctrl_name] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        }
        config[cm_node]["ros__parameters"].update(cm_controllers)
        config[(f"/{ns}/{ctrl_name}" if ns else f"/{ctrl_name}")] = {
            "ros__parameters": {
                "joints": [
                    f"{side}_thumb_metacarpal_joint",
                    f"{side}_thumb_proximal_joint",
                    f"{side}_index_proximal_joint",
                    f"{side}_middle_proximal_joint",
                    f"{side}_ring_proximal_joint",
                    f"{side}_pinky_proximal_joint",
                ],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        }

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="ros2_controllers_", delete=False
    )
    yaml.dump(config, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _build_namespaced_moveit_rviz_config(package_path, namespace):
    """为多臂场景生成带 namespace 的 RViz 配置文件（修改 Move Group Namespace）。"""
    base_rviz = package_path / "config/moveit.rviz"
    content = base_rviz.read_text(encoding="utf-8")

    ns = namespace.strip("/")
    move_group_ns = f"/{ns}" if ns else ""

    content = content.replace(
        'Move Group Namespace: ""',
        f'Move Group Namespace: "{move_group_ns}"',
    )

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".rviz", prefix="moveit_", delete=False
    )
    tmp.write(content)
    tmp.close()
    return tmp.name


def _resolve_use_sim(context) -> bool:
    """是否启动 ros2_control 仿真链。auto 时 follow:=true 为真机，不启仿真。"""
    use_sim = LaunchConfiguration("use_sim").perform(context)
    follow = LaunchConfiguration("follow").perform(context)
    if use_sim == "auto":
        return follow != "true"
    return use_sim == "true"


def _build_moveit(context):
    """组装本次 launch 需要启动的全部节点与子 launch。"""
    namespace = LaunchConfiguration("namespace").perform(context)
    arm_type = LaunchConfiguration("arm_type").perform(context)
    effector_type = LaunchConfiguration("effector_type").perform(context)
    revo2_type = LaunchConfiguration("revo2_type").perform(context)
    use_sim = _resolve_use_sim(context)
    moveit_config = build_moveit_config(context)
    package_path = moveit_config.package_path

    actions = []

    # 1. 虚拟关节 TF（base_link → world）
    virtual_joints_launch = package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch))
            )
        )

    # 2. robot_state_publisher（关节角 → TF）
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/rsp.launch.py")
            )
        )
    )

    # 3. move_group 规划节点（核心）
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/move_group.launch.py")
            ),
            launch_arguments={
                'capabilities': LaunchConfiguration('capabilities'),
                'disable_capabilities': LaunchConfiguration('disable_capabilities'),
            }.items(),#cjl: 添加 capabilities 和 disable_capabilities 参数
        )
    )

    # 4. 控制门控（可选）：仅在 Execute 执行轨迹时打开 control_enable 服务
    actions.append(
        Node(
            package="agx_arm_moveit",
            executable="agx_arm_control_gate",
            output="screen",
            parameters=[
                {
                    "status_topics": [
                        "arm_controller/follow_joint_trajectory/_action/status",
                    ],
                    "gate_service_name": LaunchConfiguration("control_gate_service"),
                }
            ],
            condition=IfCondition(LaunchConfiguration("auto_control_gate")),
        )
    )

    # 5. RViz 可视化（Plan / Execute 界面）
    custom_rviz = LaunchConfiguration("rviz_config").perform(context).strip()
    if custom_rviz:
        rviz_config_path = custom_rviz
    else:
        rviz_config_path = _build_namespaced_moveit_rviz_config(package_path, namespace)
    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(package_path / "launch/moveit_rviz.launch.py")
                    ),
                    launch_arguments={
                        "rviz_config": rviz_config_path,
                    }.items(),
                    condition=IfCondition(LaunchConfiguration("use_rviz")),
                )
            ],
        )
    )

    # 6. 场景数据库（可选，默认关闭）
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # 7–8. ros2_control 仿真链（真机模式 use_sim:=false 时不启动，避免与 agx_arm_ctrl 争抢话题）
    if use_sim:
        ros2_controllers_yaml = _build_ros2_controllers_file(
            arm_type, effector_type, revo2_type, namespace
        )
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    moveit_config.robot_description,
                    ros2_controllers_yaml,
                ],
                remappings=[("joint_states", LaunchConfiguration("control_topic"))],
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(package_path / "launch/spawn_controllers.launch.py")
                )
            )
        )

    # GroupAction：统一 namespace 与 robot_description remap
    return [
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                SetRemap(src="/robot_description", dst="robot_description"),
                *actions,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            # ── 机械臂与末端参数 ──
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for this arm instance (e.g. arm1).",
            ),
            DeclareLaunchArgument(
                "arm_type",
                default_value="piper",
                choices=ALL_ARM_TYPES,
                description="Arm type.",
            ),
            DeclareLaunchArgument(
                "effector_type",
                default_value="none",
                choices=ALL_EFFECTOR_TYPES,
                description="Effector type.",
            ),
            DeclareLaunchArgument(
                "revo2_type",
                default_value="left",
                choices=ALL_REVO2_TYPES,
                description="Revo2 side (used when effector_type is revo2).",
            ),
            DeclareLaunchArgument(
                "tcp_offset",
                default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                description="TCP offset [x, y, z, rx, ry, rz] in meters/radians.",
            ),
            # follow=true：MoveIt 订阅真机 feedback；false：用仿真控制话题
            DeclareLaunchArgument(
                "follow",
                default_value="false",
                choices=["true", "false"],
                description="Follow real arm state. "
                "true: move_group subscribes to feedback_topic; "
                "false: subscribes to control_topic (mock hardware).",
            ),
            DeclareLaunchArgument(
                "use_sim",
                default_value="auto",
                choices=["auto", "true", "false"],
                description=(
                    "Start ros2_control simulation stack. "
                    "auto: false when follow:=true, true when follow:=false. "
                    "Real hardware should use follow:=true use_sim:=false."
                ),
            ),
            # ── 可选功能开关 ──
            DeclareBooleanLaunchArg(
                "db",
                default_value=False,
                description="By default, we do not start a database (it can be large)",
            ),
            DeclareBooleanLaunchArg(
                "debug",
                default_value=False,
                description="By default, we are not in debug mode",
            ),
            DeclareBooleanLaunchArg("use_rviz", default_value=True),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Optional RViz config file. Empty uses agx_arm_moveit/config/moveit.rviz.",
            ),
            DeclareBooleanLaunchArg(
                "auto_control_gate",
                default_value=False,
                description="Automatically gate /control commands during execute only.",
            ),
            DeclareLaunchArgument(
                "control_gate_service",
                default_value="control_enable",
                description="SetBool gate service for agx_arm_control_gate (maps to gate_service_name).",
            ),
            DeclareLaunchArgument(
                "capabilities",
                default_value="",
                description="Comma-separated move_group capabilities to load.",
            ),#cjl: 添加 capabilities 和 disable_capabilities 参数
            DeclareLaunchArgument(
                "disable_capabilities",
                default_value="",
                description="Comma-separated move_group capabilities to disable.",
            ),
            OpaqueFunction(function=_build_moveit),
        ]
    )
