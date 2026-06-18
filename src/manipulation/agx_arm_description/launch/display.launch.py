"""机械臂 URDF 可视化启动文件（RViz + TF）。

根据 arm_type / effector_type 自动选择 xacro/urdf 模型，启动：
  - robot_state_publisher  : 关节角 → TF 树
  - rviz2                  : 3D 可视化
  - joint_state_publisher  : 可选，滑块发 control/joint_states（control:=true）
  - static_transform_publisher : 可选，tcp_offset 非零时发布 tcp_link

启动方式：
  ros2 launch agx_arm_description display.launch.py
  ros2 launch agx_arm_description display.launch.py \\
      arm_type:=piper_l effector_type:=agx_gripper follow:=true gui:=false

多臂 namespace：
  设置 namespace:=arm1 时，自动调整 frame_prefix 与 RViz Fixed Frame。
"""

import ast
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 支持的机械臂型号列表
ARM_TYPES = ('nero', 'piper', 'piper_h', 'piper_l', 'piper_x')

# 各型号裸臂 URDF 相对路径（位于 agx_arm_urdf/ 下）
ROBOT_URDF_MAP = {
    arm_type: f'{arm_type}/urdf/{arm_type}_description.urdf'
    for arm_type in ARM_TYPES
}

# 带 AGX 夹爪的 xacro 路径
ROBOT_WITH_GRIPPER_URDF_MAP = {
    arm_type: f'{arm_type}/urdf/{arm_type}_with_gripper_description.xacro'
    for arm_type in ARM_TYPES
}

# 带 Revo2 灵巧手的 xacro 路径（分左右手）
ROBOT_WITH_REVO2_URDF_MAP = {
    arm_type: {
        side: f'{arm_type}/urdf/{arm_type}_with_{side}_revo2_description.xacro'
        for side in ('left', 'right')
    }
    for arm_type in ARM_TYPES
}


def _resolve_custom_model_path(pkg_path, custom_model):
    """解析用户自定义模型路径。

    优先在包内 agx_arm_urdf/ 下查找；否则按用户绝对/相对路径解析。
    """
    custom_path = pkg_path / 'agx_arm_urdf' / custom_model
    if custom_path.exists() and custom_path.is_file():
        return str(custom_path)

    expanded = Path(custom_model).expanduser()
    return str(expanded)


def _resolve_builtin_model_path(arm_type, effector_type, revo2_type, pkg_path):
    """根据臂型与末端类型，选择内置 URDF/xacro 文件。"""
    if effector_type == 'agx_gripper':
        relative_path = ROBOT_WITH_GRIPPER_URDF_MAP[arm_type]
    elif effector_type == 'revo2':
        relative_path = ROBOT_WITH_REVO2_URDF_MAP[arm_type][revo2_type]
    else:
        relative_path = ROBOT_URDF_MAP[arm_type]
    return str(pkg_path / 'agx_arm_urdf' / relative_path)


def _flange_link(arm_type):
    """返回法兰 link 名称：nero 为 7 轴用 link7，其余 6 轴用 link6。"""
    return 'link7' if arm_type == 'nero' else 'link6'


def _rviz_config_path_with_ns(template_path: str, ns: str) -> str:
    """多臂场景：把 RViz 配置中的 Fixed Frame / TF Prefix 改为带 namespace 的版本。

    返回临时 rviz 文件路径（不自动删除，进程结束后由系统清理）。
    """
    if not ns:
        return template_path
    text = Path(template_path).read_text(encoding='utf-8')
    text = text.replace('Fixed Frame: base_link', f'Fixed Frame: {ns}/base_link')
    text = text.replace('TF Prefix: ""', f'TF Prefix: "{ns}"')
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.rviz',
        prefix=f'display_{ns}_',
        delete=False,
        encoding='utf-8',
    )
    tmp.write(text)
    tmp.close()
    return tmp.name


def resolve_model_path(context, *args, **kwargs):
    """OpaqueFunction 回调：在 launch 解析阶段动态创建节点列表。

    根据运行时参数决定模型路径、关节话题来源、是否发布 tcp_link 等。
    """
    namespace = LaunchConfiguration('namespace').perform(context)
    arm_type = LaunchConfiguration('arm_type').perform(context)
    effector_type = LaunchConfiguration('effector_type').perform(context)
    revo2_type = LaunchConfiguration('revo2_type').perform(context)
    custom_model = LaunchConfiguration('custom_model').perform(context)
    follow = LaunchConfiguration('follow').perform(context)
    control = LaunchConfiguration('control').perform(context)
    control_topic = LaunchConfiguration('control_topic').perform(context)
    feedback_topic = LaunchConfiguration('feedback_topic').perform(context)
    tcp_offset = ast.literal_eval(
        LaunchConfiguration('tcp_offset').perform(context)
    )
    pkg_path = get_package_share_path('agx_arm_description')

    # 确定机器人描述文件路径
    if custom_model:
        model_path = _resolve_custom_model_path(pkg_path, custom_model)
    else:
        model_path = _resolve_builtin_model_path(
            arm_type, effector_type, revo2_type, pkg_path
        )

    # xacro 经 Command 在运行时展开为 robot_description 字符串
    robot_description = ParameterValue(Command(['xacro ', model_path]), value_type=str)

    # follow=true 订阅真机反馈；false 订阅控制话题（仿真/滑块）
    state_joint_topic = str(feedback_topic) if follow == 'true' else str(control_topic)

    # 多臂：RSP 用 frame_prefix 区分 TF；RViz 用 Fixed Frame + TF Prefix
    ns = namespace.strip()
    frame_prefix = f'{ns}/' if ns else ''
    rsp_params = {'robot_description': robot_description}
    if frame_prefix:
        rsp_params['frame_prefix'] = frame_prefix

    # 关节角 → link TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[rsp_params],
        remappings=[('joint_states', state_joint_topic)]
    )

    # 无 GUI 时的关节滑块发布器（control=true 时启用）
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
        remappings=[('joint_states', str(control_topic))]
    )

    # 带 GUI 滑块的关节发布器
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=namespace,
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
        remappings=[('joint_states', str(control_topic))]
    )

    rviz_cfg = _rviz_config_path_with_ns(
        LaunchConfiguration('rvizconfig').perform(context), ns)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        output='screen',
        arguments=['-d', rviz_cfg],
        remappings=[('/robot_description', 'robot_description')],
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
    ]

    # control=true 时增加滑块节点，用于手动发 control/joint_states
    if control == 'true':
        nodes = [
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            *nodes,
        ]

    # tcp_offset 非零时，发布法兰 → tcp_link 的静态 TF
    if any(v != 0.0 for v in tcp_offset):
        flange = _flange_link(arm_type)
        flange_frame = f'{frame_prefix}{flange}' if frame_prefix else flange
        tcp_frame = f'{frame_prefix}tcp_link' if frame_prefix else 'tcp_link'
        x, y, z, rx, ry, rz = tcp_offset
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                namespace=namespace,
                arguments=[
                    '--x', str(x), '--y', str(y), '--z', str(z),
                    '--roll', str(rx), '--pitch', str(ry), '--yaw', str(rz),
                    '--frame-id', flange_frame, '--child-frame-id', tcp_frame,
                ],
            )
        )

    return nodes


def generate_launch_description():
    """声明全部 launch 参数，并通过 OpaqueFunction 延迟创建节点。"""
    urdf_tutorial_path = get_package_share_path('agx_arm_description')
    default_rviz_config_path = urdf_tutorial_path / 'rviz/display.rviz'

    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Namespace for topics/nodes; when set, TF frame_prefix and RViz '
                      'RobotModel (Fixed Frame / TF Prefix) are adjusted automatically.',
    )
    arm_type_arg = DeclareLaunchArgument(
        name='arm_type',
        default_value='piper',
        choices=list(ROBOT_URDF_MAP.keys()),
        description='Robotic arm type (e.g. nero, piper, piper_x, piper_l, piper_h).'
    )
    custom_model_arg = DeclareLaunchArgument(
        name='custom_model',
        default_value='',
        description='Optional custom model path. Prefer absolute path, or relative path under agx_arm_urdf/. If set, arm_type and effector_type are ignored.'
    )
    effector_type_arg = DeclareLaunchArgument(
        name='effector_type',
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
    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200',
        description='Publishing rate for the AGX Arm node.'
    )
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    follow_arg = DeclareLaunchArgument(name='follow', default_value='false', choices=['true', 'false'],
                                       description='Flag to enable follow mode')
    control_arg = DeclareLaunchArgument(
        name='control',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable publishing control topics.',
    )
    control_topic_arg = DeclareLaunchArgument(
        name='control_topic',
        default_value='control/joint_states',
        description='Topic to publish joint slider targets (from joint_state_publisher_gui).',
    )
    feedback_topic_arg = DeclareLaunchArgument(
        name='feedback_topic',
        default_value='feedback/joint_states',
        description='Topic to subscribe as the feedback joint states (used when follow:=true).',
    )
    tcp_offset_arg = DeclareLaunchArgument(
        'tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='TCP offset [x, y, z, rx, ry, rz] in meters/radians. '
                    'When non-zero, a tcp_link TF is published relative to the flange.',
    )

    return LaunchDescription([
        namespace_arg,
        arm_type_arg,
        custom_model_arg,
        effector_type_arg,
        revo2_type_arg,
        pub_rate_arg,
        gui_arg,
        rviz_arg,
        follow_arg,
        control_arg,
        control_topic_arg,
        feedback_topic_arg,
        tcp_offset_arg,
        OpaqueFunction(function=resolve_model_path),
    ])
