# gripper_controller_node 说明

> **实现语言**：C++（`src/gripper_controller_node.cpp`）  
> **维护约定**：修改节点实现、`gripper_controller_params.yaml` 或 launch 时，同步更新本文档。

## 1. 功能

夹爪闭环控制与夹持判定（规划决策层 ID 17）：

- 订阅 `/task/gripper_cmd` 执行张开/闭合/停止
- 订阅原始夹爪反馈（默认 `/feedback/gripper_status`）
- 发布 `/gripper/status`，含力阈值与电流阈值双判据结果

判据逻辑：

| 判据 | 条件 | 说明 |
|------|------|------|
| 力阈值 | `force >= force_grasp_threshold` | 夹持力达到阈值 |
| 电流阈值 | `driver_overcurrent` 或 `force >= current_grasp_threshold` | 过流/力矩上升视为接触 |

`wait_grasp=true` 时，闭合指令会阻塞直到任一判据满足或超时。

# 流程图
发送夹爪命令
↓
监听宽度变化（是否真的闭合）
↓
监听力/电流变化（是否夹住物体）
↓
判断 grasp 成功 / 失败
↓
发布状态

## 2. 话题接口

| 方向 | 话题 | 消息 |
|------|------|------|
| 订阅 | `/task/gripper_cmd` | `agx_motion_msgs/GripperCmd` |
| 订阅 | `/gripper/raw_status`（可配置为 `/feedback/gripper_status`） | `agx_arm_msgs/GripperStatus` |
| 发布 | `/gripper/status` | `agx_motion_msgs/GripperControlStatus` |
| 发布 | `/control/gripper_joint_states` | `sensor_msgs/JointState`（仅 gripper 关节） |

控制通路：

```text
gripper_controller_node → /control/gripper_joint_states (gripper 关节 width + effort)
                        → agx_arm_ctrl → CAN 夹爪驱动
```

> 真机模式勿同时启动 ros2_control（`use_sim:=false`），否则仿真 `joint_state_broadcaster` 会向 `/control/joint_states` 广播闭合夹爪，与专用话题冲突导致反复开关。

## 3. GripperCmd 指令

| `command` | 常量 | 行为 |
|-----------|------|------|
| 0 | `CMD_OPEN` | 张开到 `open_width`（或 `target_width`） |
| 1 | `CMD_CLOSE` | 闭合到 `close_width`，可选 `wait_grasp` |
| 2 | `CMD_STOP` | 停止（当前仅更新状态为 IDLE） |

## 4. GripperControlStatus 状态

| `state` | 含义 |
|---------|------|
| `STATE_IDLE` | 空闲 |
| `STATE_MOVING` | 运动中 |
| `STATE_GRASPED` | 夹持成功（双判据之一满足） |
| `STATE_FAILED` | 失败/超时 |
| `STATE_OPEN` | 已张开到位 |

## 5. 参数

见 `config/gripper_controller_params.yaml`。

| 参数 | 默认 | 说明 |
|------|------|------|
| `open_width` | 0.07 | 张开宽度 (m)，70mm 夹爪勿超过 0.07 |
| `close_width` | 0.0 | 闭合目标宽度 (m) |
| `default_force` | 1.5 | 默认夹持力 (N) |
| `force_grasp_threshold` | 0.8 | 力判据阈值 (N) |
| `current_grasp_threshold` | 0.5 | 电流辅助判据 (N) |
| `grasp_timeout_sec` | 5.0 | 等待夹持超时 (s) |

## 6. 启动

```bash
ros2 launch agx_gripper_controller gripper_controller_node.launch.py
```

需先启动 `agx_arm_ctrl`（`effector_type:=agx_gripper`）。

## 7. 测试

```bash
# 张开
ros2 topic pub --once /task/gripper_cmd agx_motion_msgs/msg/GripperCmd \
  "{request_id: 'open', command: 0}"

# 闭合并等待夹持
ros2 topic pub --once /task/gripper_cmd agx_motion_msgs/msg/GripperCmd \
  "{request_id: 'close', command: 1, max_force: 1.5, wait_grasp: true}"

# 查看状态
ros2 topic echo /gripper/status
```

## 8. 依赖

- `agx_motion_msgs`
- `agx_arm_msgs`
- `agx_arm_ctrl`
