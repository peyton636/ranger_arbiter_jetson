# cangyi_bringup

统一系统启动入口，封装单臂控制、RViz显示、MoveIt规划三种模式。

## 1. 编译

```bash
cd ~/workspace/cangyirobot
colcon build --symlink-install --packages-up-to cangyi_bringup
source install/setup.bash
```

## 2. 启动

1) 仅控制（真机控制节点）

```bash
ros2 launch cangyi_bringup bringup.launch.py mode:=control can_port:=can0 arm_type:=piper
```

2) 控制 + RViz

```bash
ros2 launch cangyi_bringup bringup.launch.py mode:=rviz can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```

3) 控制 + MoveIt

```bash
ros2 launch cangyi_bringup bringup.launch.py mode:=moveit can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```

## 3. 常用参数

- `namespace`: 多臂隔离命名空间，例如 `arm1`
- `can_port`: CAN口，例如 `can0`
- `arm_type`: `nero | piper | piper_h | piper_l | piper_x`
- `effector_type`: `none | agx_gripper | revo2`
- `revo2_type`: `left | right`
- `follow`: MoveIt/RViz是否跟随真机反馈
- `auto_control_gate`: MoveIt执行期门控
- `control_gate_service`: 门控服务名（默认 `control_enable`）
