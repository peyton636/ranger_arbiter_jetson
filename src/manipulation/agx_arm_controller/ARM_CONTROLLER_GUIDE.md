# arm_controller_node 说明

> **实现语言**：C++（`src/arm_controller_node.cpp`）  
> **维护约定**：修改节点实现、`arm_controller_params.yaml` 或 launch 时，同步更新本文档。

## 1. 功能

轨迹下发执行与结果回传（规划决策层 ID 16）：

- 订阅 `/motion/trajectory`
- 按轨迹点向 `/control/joint_states` 下发关节指令
- 转发 `/feedback/joint_states` 到 `/joint/states`
- 执行完成后发布 `/motion/execute_result`

# 流程图

订阅上层发来的轨迹
/motion/trajectory
        ↓
逐个关节点下发给机械臂驱动
/control/joint_states
        ↓
监听真实机械臂反馈
/feedback/joint_states
        ↓
判断是否到位
        ↓
发布执行结果
/motion/execute_result

对接通路：

```text
arm_controller_node → /control/joint_states → agx_arm_ctrl_single_node → CAN 硬件
                   ← /feedback/joint_states ←
```

仿真模式下可对接 `ros2_control` 的 `JointTrajectoryController`（经 remapping 到 `control/joint_states`）。

## 2. 话题接口


| 方向  | 话题                       | 消息                                 |
| --- | ------------------------ | ---------------------------------- |
| 订阅  | `/motion/trajectory`     | `agx_motion_msgs/MotionTrajectory` |
| 订阅  | `/feedback/joint_states` | `sensor_msgs/JointState`           |
| 发布  | `/control/joint_states`  | `sensor_msgs/JointState`           |
| 发布  | `/joint/states`          | `sensor_msgs/JointState`           |
| 发布  | `/motion/execute_result` | `agx_motion_msgs/ExecuteResult`    |


## 3. 执行逻辑

1. 收到 `MotionTrajectory` 后解析 `trajectory.joint_trajectory` 各路径点
2. 逐点发布 `JointState` 到 `/control/joint_states`
3. 轮询 `/feedback/joint_states`，关节误差小于 `goal_tolerance` 视为到位
4. 全部点完成后发布 `success=true` 的 `ExecuteResult`

同一时刻仅处理一条轨迹；忙时回传 `busy` 错误。

## 4. 参数

见 `config/arm_controller_params.yaml`。


| 参数                  | 默认   | 说明               |
| ------------------- | ---- | ---------------- |
| `control_rate_hz`   | 50.0 | 轨迹点下发频率          |
| `goal_tolerance`    | 0.02 | 关节到位容差 (rad 或 m) |
| `reach_timeout_sec` | 5.0  | 单点到位超时 (s)       |


## 5. 启动

```bash
ros2 launch agx_arm_controller arm_controller_node.launch.py
```

真机需先启动 `agx_arm_ctrl`。

## 6. 依赖

- `agx_motion_msgs`
- `agx_arm_ctrl`（真机执行）

## 7. 扩展建议

- 支持 `trajectory_msgs/JointTrajectory` 时间插值，而非逐点阻塞
- 增加急停订阅，执行中可 abort 并发布 `STATUS_ABORTED`

