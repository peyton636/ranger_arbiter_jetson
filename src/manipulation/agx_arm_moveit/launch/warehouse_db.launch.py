"""启动 MoveIt 场景数据库（可选，保存/加载规划场景）。

由 demo.launch.py 在 db:=true 时包含调用。数据库体积较大，日常调试建议保持默认关闭。
启用后可保存碰撞体、物体位姿等规划场景，下次启动时恢复。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from moveit_configs_utils.launches import generate_warehouse_db_launch

from _moveit_config_builder import build_moveit_config, declare_common_args


def _launch(context):
    moveit_config = build_moveit_config(context)
    return list(generate_warehouse_db_launch(moveit_config).entities)


def generate_launch_description():
    return LaunchDescription(declare_common_args() + [OpaqueFunction(function=_launch)])
