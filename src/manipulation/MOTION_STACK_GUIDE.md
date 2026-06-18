# 规划决策层运动栈（manipulation）指导手册

> **维护约定**：修改 `manipulation/` 下节点、launch、参数或测试流程时，**同步更新本文档**。
>
> 对应架构表 ID 15–17：ROS 包 `agx_motion_planner` / `agx_arm_controller` / `agx_gripper_controller`（节点名仍为 `/motion_planner_node` 等）

---

## 1. 包结构

源码位于 `cangyirobot/src/manipulation/`（消息包在 `src/interfaces/`）：


| 源码目录                     | ROS 包名                    | 节点名                        | 职责              |
| ------------------------ | ------------------------- | -------------------------- | --------------- |
| `agx_motion_msgs`        | `agx_motion_msgs`         | —                          | 三节点共用消息         |
| `agx_motion_planner`     | `agx_motion_planner`      | `/motion_planner_node`     | MoveIt2 规划与执行监控 |
| `agx_arm_controller`     | `agx_arm_controller`      | `/arm_controller_node`     | 轨迹下发、执行、结果回传    |
| `agx_gripper_controller` | `agx_gripper_controller`  | `/gripper_controller_node` | 夹爪闭环与夹持判定       |


### 1.1 数据流

```text
task_fsm_node（后续实现）
    │
    ├─ /motion/plan_request ──► motion_planner_node ──► /motion/trajectory
    │                              │                      /motion/execute_feedback
    │                              │◄── /motion/execute_result
    │
    ├─ /task/gripper_cmd ────────► gripper_controller_node ──► /gripper/status
    │                              ▲
    │                              └── /feedback/gripper_status
    │
    └─ /grasp/selected_pose ─────► motion_planner_node

/motion/trajectory ──► arm_controller_node ──► /control/joint_states（臂关节）
                         │                         ↓
                         │              agx_arm_ctrl（真机）
                         ├── /joint/states（转发 /feedback/joint_states）
                         └── /motion/execute_result

/task/gripper_cmd ──► gripper_controller_node ──► /control/gripper_joint_states
                                                    ↓
                                         agx_arm_ctrl（真机夹爪）

真机模式（`follow:=true use_sim:=false`）：**不启动** ros2_control / joint_state_broadcaster。
仿真模式（`follow:=false use_sim:=true`）：ros2_control 驱动 `/control/joint_states`。
```

### 1.2 依赖的基础包（不可删除）


| 包                     | 作用                          |
| --------------------- | --------------------------- |
| `agx_arm_description` | 机器人 URDF 模型                 |
| `agx_arm_moveit`      | MoveIt 配置、`move_group`、RViz |
| `agx_arm_ctrl`        | 真机 CAN 驱动                   |
| `agx_arm_msgs`        | 夹爪/臂状态消息                    |


---

## 2. 编译与环境

每个终端执行：

```bash
cd ~/cangyi_robot/cangyirobot
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  agx_arm_msgs agx_motion_msgs \
  agx_arm_ctrl agx_arm_description agx_arm_moveit \
  agx_arm_controller agx_gripper_controller agx_motion_planner
source install/setup.bash
```

---

## 3. 仿真测试（含 RViz）

仿真**不需要**启动 `agx_arm_ctrl`，默认 `follow:=false use_sim:=true`（或 `use_sim:=auto`），关节状态由 `ros2_control` 经 `/control/joint_states` 更新，**RViz 会显示机器人模型与规划轨迹**。

### 3.1 启动

**终端 1（MoveIt + RViz + 三节点）：**

```bash
cd ~/cangyi_robot/agx_arm_ros-ros2
source install/setup.bash

ros2 launch agx_motion_planner motion_stack.launch.py \
    arm_type:=piper_l \
    effector_type:=agx_gripper \
    follow:=false
```

等待终端出现：

- MoveIt：`You can start planning now!`
- 三节点：`motion_planner_node ready` / `arm_controller_node ready` / `gripper_controller_node ready`
- **RViz 窗口自动弹出**，显示机械臂模型

**终端 2（测试命令）：**

```bash
source install/setup.bash
```

### 3.2 RViz 中应看到的内容


| 项目                | 说明                                                      |
| ----------------- | ------------------------------------------------------- |
| 机器人模型             | `piper_l` + 夹爪，可拖动视角                                    |
| MotionPlanning 面板 | 可选规划组 `arm` / `gripper`（本栈主要通过话题测试）                     |
| 规划轨迹              | 发送 `execute: false` 的 PlanRequest 后，可在 RViz 看到橙色/绿色轨迹预览 |
| 执行动画              | `execute: true` 后，模型沿轨迹运动（走 ros2_control 仿真链）           |


> 仿真模式下无 `/feedback/gripper_status`，**夹爪闭环测试（阶段 S4）可跳过**，或仅观察 `/control/joint_states` 是否有 gripper 关节指令。

### 3.3 健康检查

```bash
ros2 node list | grep -E 'motion_planner|arm_controller|gripper_controller|move_group'
ros2 topic list | grep -E 'motion/|task/gripper|joint/states'
```

### 3.4 测试步骤

#### 阶段 S1：仅规划（臂不动，RViz 看轨迹）

监听反馈：

```bash
ros2 topic echo /motion/execute_feedback
```

规划到 `grasp_ready`：

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'sim_plan_ready', plan_type: 1, group_name: 'arm', named_target: 'grasp_ready', execute: false, max_velocity_scaling: 0.3, max_acceleration_scaling: 0.3}"
```

规划到 `home`：

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'sim_plan_home', plan_type: 1, group_name: 'arm', named_target: 'home', execute: false, max_velocity_scaling: 0.3, max_acceleration_scaling: 0.3}"
```

**通过标准：**

- `/motion/execute_feedback` → `status: 4`（SUCCEEDED），`message: plan only`
- RViz 中可见规划路径
- 机器人模型**不应**运动

#### 阶段 S2：规划并执行（RViz 看动画）

同时监听：

```bash
ros2 topic echo /motion/execute_feedback &
ros2 topic echo /motion/execute_result
```

执行到 `grasp_ready`：

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'sim_exec_ready', plan_type: 1, group_name: 'arm', named_target: 'grasp_ready', execute: true, max_velocity_scaling: 0.3, max_acceleration_scaling: 0.3}"
```

执行回 `home`：

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'sim_exec_home', plan_type: 1, group_name: 'arm', named_target: 'home', execute: true, max_velocity_scaling: 0.3, max_acceleration_scaling: 0.3}"
```

**通过标准：**

- RViz 模型沿轨迹运动
- `/motion/execute_result` → `success: true`
- `/motion/execute_feedback` → `STATUS_SUCCEEDED`

#### 阶段 S3：关节空间 / 笛卡尔规划（可选）

关节目标：

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'sim_joint', plan_type: 0, group_name: 'arm', joint_goal: [0.0, 0.35, -0.55, 0.0, 0.45, 0.0], execute: false, max_velocity_scaling: 0.3, max_acceleration_scaling: 0.3}"
```

TCP 方向小位移（先 `execute: false` 预览）：

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'sim_cart', plan_type: 3, group_name: 'arm', cartesian_direction: {header: {frame_id: 'tcp_link'}, vector: {x: 0.0, y: 0.0, z: -0.02}}, cartesian_min_dist: 0.0, cartesian_max_dist: 0.02, cartesian_step_size: 0.005, execute: false, max_velocity_scaling: 0.2, max_acceleration_scaling: 0.2}"
```

### 3.5 仿真测试顺序小结


| 顺序  | 内容             | 是否动模型        |
| --- | -------------- | ------------ |
| S1  | 仅规划 + RViz 看轨迹 | 否            |
| S2  | 规划并执行          | 是（RViz 动画）   |
| S3  | 关节/笛卡尔（可选）     | 视 execute 而定 |


---

## 4. 真机测试

真机**必须先**启动 `agx_arm_ctrl`，并设置 **`follow:=true use_sim:=false`**，使 MoveIt / RViz 订阅 `/feedback/joint_states` 与真机姿态一致，且**不启动 ros2_control 仿真**（否则 `joint_state_broadcaster` 会以 200Hz 向 `/control/joint_states` 下发闭合夹爪指令，与 `gripper_controller` 冲突，出现夹爪反复开关）。

> **推荐长期真机用法**：`motion_stack_real.launch.py`（已固定 `follow:=true use_sim:=false`）。

### 4.1 安全准备

- 工作空间无障碍，急停可用
- CAN 线连接正常
- 测试速度缩放建议 **0.1 ~ 0.15**
- 每个终端均 `source install/setup.bash`

### 4.2 启动

**终端 1（CAN + 臂驱动）：**

```bash
cd ~/cangyi_robot/agx_arm_ros-ros2
source install/setup.bash

bash scripts/can_activate.sh can2 1000000 "1-2.3:1.0"
timeout 3 candump can2 | head    # 确认有 CAN 报文

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:=can2 \
    arm_type:=piper_l \
    effector_type:=agx_gripper \
    auto_enable:=true
```

**终端 2（MoveIt + RViz + 三节点，纯真机、无仿真）：**

```bash
source install/setup.bash

# 方式 A（推荐）
ros2 launch agx_motion_planner motion_stack_real.launch.py \
    arm_type:=piper_l \
    effector_type:=agx_gripper

# 方式 B（等价）
ros2 launch agx_motion_planner motion_stack.launch.py \
    follow:=true \
    use_sim:=false \
    arm_type:=piper_l \
    effector_type:=agx_gripper
```

启动后确认 **没有** `ros2_control_node`、`joint_state_broadcaster`：

```bash
ros2 node list | grep -E 'ros2_control|broadcaster'   # 应无输出
ros2 topic info /control/joint_states -v                # 发布者应为 arm_controller_node（执行轨迹时）
```

**终端 3（测试命令）：**

```bash
source install/setup.bash
```

### 4.3 阶段 0：驱动与反馈（只读，不动臂）

```bash
ros2 topic hz /feedback/joint_states          # 应 ~200Hz
ros2 topic echo /feedback/arm_status --once
ros2 topic echo /feedback/gripper_status --once
ros2 topic echo /feedback/tcp_pose --once
```

**通过标准：**

- 无持续 `Agx_arm is not connected` 报错
- 关节角、TCP、夹爪状态均有数据
- RViz 模型与真机姿态一致（`follow:=true`）

### 4.4 阶段 1：节点检查

```bash
ros2 node list | grep -E 'motion_planner|arm_controller|gripper_controller|agx_arm_ctrl|move_group'
```

### 4.5 阶段 2：夹爪单测（最安全）

开门控：

```bash
ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"
```

监听：

```bash
ros2 topic echo /gripper/status
```

张开：

```bash
ros2 topic pub --once /task/gripper_cmd agx_motion_msgs/msg/GripperCmd \
  "{request_id: 'hw_open', command: 0, max_force: 1.0, wait_grasp: false}"
```

闭合 + 判持：

```bash
ros2 topic pub --once /task/gripper_cmd agx_motion_msgs/msg/GripperCmd \
  "{request_id: 'hw_grasp', command: 1, max_force: 1.5, wait_grasp: true}"
```

**通过标准：** 夹爪有物理动作；`/gripper/status` 有响应（`state: 4` OPEN / `state: 2` GRASPED）

**链路自检（夹爪不动时必查）：**

```bash
ros2 topic info /control/gripper_joint_states -v
# Subscription 必须包含 agx_arm_ctrl_single_node；若为 0，请重启 agx_arm_ctrl 终端

ros2 topic echo /feedback/gripper_status --once
# homing_status: true 为佳；false 时宽度读数可能不准，需闭合后标零
```

### 4.6 阶段 3：仅规划（不执行）

```bash
ros2 topic echo /motion/execute_feedback
```

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'hw_plan', plan_type: 1, group_name: 'arm', named_target: 'grasp_ready', execute: false, max_velocity_scaling: 0.2, max_acceleration_scaling: 0.2}"
```

**通过标准：** 反馈 SUCCEEDED；RViz 可见轨迹；**真机不动**

### 4.7 阶段 4：低速执行（验证完整链路）

监听：

```bash
ros2 topic echo /motion/execute_feedback &
ros2 topic echo /motion/execute_result
```

确认门控：

```bash
ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"
```

低速回 home：

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'hw_home', plan_type: 1, group_name: 'arm', named_target: 'home', execute: true, max_velocity_scaling: 0.1, max_acceleration_scaling: 0.1}"
```

低速到 grasp_ready：

```bash
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'hw_ready', plan_type: 1, group_name: 'arm', named_target: 'grasp_ready', execute: true, max_velocity_scaling: 0.1, max_acceleration_scaling: 0.1}"
```

**通过标准（完整链路）：**

```text
/motion/plan_request → motion_planner_node → /motion/trajectory
  → arm_controller_node → /control/joint_states → agx_arm_ctrl → 真机动
  → /motion/execute_result (success: true)
```

### 4.8 阶段 5：mini pick 手测（确认无障碍后再做）

速度保持 **0.15** 以下：

```bash
# 1. 张开夹爪
ros2 topic pub --once /task/gripper_cmd agx_motion_msgs/msg/GripperCmd \
  "{request_id: 'p1', command: 0, max_force: 1.0, wait_grasp: false}"

# 2. 到 grasp_ready
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'p2', plan_type: 1, group_name: 'arm', named_target: 'grasp_ready', execute: true, max_velocity_scaling: 0.15, max_acceleration_scaling: 0.15}"

# 3. 到 pre_grasp
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'p3', plan_type: 1, group_name: 'arm', named_target: 'pre_grasp', execute: true, max_velocity_scaling: 0.15, max_acceleration_scaling: 0.15}"

# 4. 闭合夹爪
ros2 topic pub --once /task/gripper_cmd agx_motion_msgs/msg/GripperCmd \
  "{request_id: 'p4', command: 1, max_force: 1.5, wait_grasp: true}"

# 5. 抬起 retreat
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'p5', plan_type: 1, group_name: 'arm', named_target: 'retreat', execute: true, max_velocity_scaling: 0.15, max_acceleration_scaling: 0.15}"

# 6. 回 home
ros2 topic pub --once /motion/plan_request agx_motion_msgs/msg/PlanRequest \
  "{request_id: 'p6', plan_type: 1, group_name: 'arm', named_target: 'home', execute: true, max_velocity_scaling: 0.15, max_acceleration_scaling: 0.15}"
```

### 4.9 急停与门控

```bash
# 急停
ros2 service call /emergency_stop std_srvs/srv/Empty

# 关闭控制门（阻止后续运动）
ros2 service call /control_enable std_srvs/srv/SetBool "{data: false}"
```

### 4.10 真机测试顺序小结


| 顺序  | 阶段                    | 是否动臂 | 风险  |
| --- | --------------------- | ---- | --- |
| 0   | CAN + 反馈只读            | 否    | 低   |
| 1   | 节点 + RViz 对齐          | 否    | 低   |
| 2   | 夹爪开/合                 | 仅夹爪  | 低   |
| 3   | 仅规划                   | 否    | 低   |
| 4   | 低速 home / grasp_ready | 是    | 中   |
| 5   | mini pick 流程          | 是    | 中高  |


---

## 5. 仿真 vs 真机 对照


| 项目                  | 仿真                                    | 真机                               |
| ------------------- | ------------------------------------- | -------------------------------- |
| `agx_arm_ctrl`      | 不需要                                   | **必须**                           |
| `follow`            | `false`（默认）                           | `**true`**                       |
| 关节状态来源              | `/control/joint_states`（ros2_control） | `/feedback/joint_states`（CAN 反馈） |
| RViz                | 显示仿真模型与轨迹                             | 显示真机跟随模型与轨迹                      |
| 门控 `control_enable` | 一般不需要                                 | **执行前必须 true**                   |
| 夹爪闭环                | 无硬件反馈，可跳过                             | 完整测试                             |
| 推荐速度缩放              | 0.3                                   | 0.1 ~ 0.15                       |


---

## 6. 故障排查


| 现象                              | 可能原因                 | 处理                                                |
| ------------------------------- | -------------------- | ------------------------------------------------- |
| `Unable to find node`           | 未 launch 或终端未 source | `source install/setup.bash`；确认 launch 终端在跑        |
| RViz 与真机姿态不一致                   | 未设 `follow:=true`    | 重启 motion_stack 并加 `follow:=true`                 |
| 规划失败                            | MoveIt 未就绪 / 位姿不可达   | 看 `motion_planner_node` 终端日志                      |
| 规划成功但臂不动                        | 门控未开 / CAN 未连        | `control_enable true`；查 `feedback/joint_states`   |
| `execute_result success: false` | 到位超时                 | 查 `/feedback/joint_states`；调大 `reach_timeout_sec` |
| `busy: previous trajectory running` | 上一条轨迹仍在执行 / `executing_` 未复位 | 等上一条结束；勿连续发 `execute: true`；重启 motion_stack |
| 仿真 `joint goal not reached in time` | 仿真链路由 `ros2_control` action 驱动，非逐点 JointState | 确认 `follow:=false` 且 arm_controller 已启用 `use_ros2_control_action`；重启 motion_stack |
| 仿真执行失败 / 一直 busy          | 上一条轨迹未结束 / `executing_` 未复位 | 等上一条结束；重启 motion_stack |
| 夹爪无反应                           | 驱动未启                 | 确认 `agx_arm_ctrl` + `effector_type:=agx_gripper`  |
| 夹爪反复开关 / `open timeout`        | 真机仍启了 ros2_control 仿真 | 改用 `use_sim:=false` 或 `motion_stack_real.launch.py`；重启全部节点 |
| 夹爪 `open timeout` / width 不变     | 未使能 / 未标零          | `control_enable true`；闭合后标零；查 `homing_status` |
| 夹爪 width 始终 ~0.0001             | homing 未完成            | `candump can2,2A8:7FF` 状态字节应含 bit7=1 |


---

## 7. 单独启动（排查用）

```bash
# 真机（无 ros2_control）
ros2 launch agx_motion_planner motion_planner_node.launch.py \
    follow:=true use_sim:=false arm_type:=piper_l effector_type:=agx_gripper
ros2 launch agx_arm_controller arm_controller_node.launch.py follow:=true
ros2 launch agx_gripper_controller gripper_controller_node.launch.py
```

---

## 8. 与 MTC 的关系

- 本栈替代 MTC 在**操纵任务编排**上的角色；完整 pick-place 由上层 `task_fsm_node` 按步骤发送 `PlanRequest` + `GripperCmd`。
- `motion_planner_node` 负责**单次**规划；多段序列不在本包内硬编码。

---

## 9. 各包详细文档


| 文档   | 路径                                                                |
| ---- | ----------------------------------------------------------------- |
| 规划节点 | `manipulation/agx_motion_planner/MOTION_PLANNER_GUIDE.md`         |
| 臂控节点 | `manipulation/agx_arm_controller/ARM_CONTROLLER_GUIDE.md`         |
| 夹爪节点 | `manipulation/agx_gripper_controller/GRIPPER_CONTROLLER_GUIDE.md` |


---

## 10. PlanRequest 速查


| `plan_type` | 含义    | 主要字段                                                             |
| ----------- | ----- | ---------------------------------------------------------------- |
| 0           | 关节空间  | `joint_goal`                                                     |
| 1           | 命名位姿  | `named_target`（`home` / `grasp_ready` / `pre_grasp` / `retreat`） |
| 2           | 笛卡尔位姿 | `pose_goal` 或 `/grasp/selected_pose`                             |
| 3           | 笛卡尔方向 | `cartesian_direction` + `cartesian_max_dist`                     |



| `GripperCmd.command` | 含义  |
| -------------------- | --- |
| 0                    | 张开  |
| 1                    | 闭合  |
| 2                    | 停止  |


