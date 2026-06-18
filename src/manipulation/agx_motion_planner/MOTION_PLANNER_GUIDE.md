# motion_planner_node 说明

> **实现语言**：C++（`src/motion_planner_node.cpp`）  
> **维护约定**：修改节点实现、`motion_planner_params.yaml` 或 launch 时，同步更新本文档。

## 1. 功能

MoveIt2 规划与执行监控（规划决策层 ID 15）：

- 订阅 `/motion/plan_request`，调用 MoveIt `MoveGroupInterface` 规划
- 发布 `/motion/trajectory` 供 `arm_controller_node` 执行
- 发布 `/motion/execute_feedback` 反馈规划/执行进度
- 订阅 `/motion/execute_result` 等待臂控执行完成（`execute=true` 时）

可选输入：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/grasp/selected_pose` | `geometry_msgs/PoseStamped` | 感知给出的抓取位姿；`PLAN_TYPE_POSE` 且 `pose_goal` 为空时使用 |
| `/joint/states` | `sensor_msgs/JointState` | 当前关节状态（预留，供后续碰撞检查扩展） |

# 规划层
MoveIt规划轨迹
↓
MotionTrajectory
↓
/motion/trajectory

# 执行层
/motion/trajectory
↓
拆成 JointState
↓
/control/joint_states
↓
机械臂
↓
/feedback/joint_states
↓
判断到位
↓
/motion/execute_result

# 抓夹层
GripperCmd
↓
/control/joint_states (gripper)
↓
raw_status（力/电流）
↓
判断 grasp 成功
↓
status


## 2. 话题接口

| 方向 | 话题 | 消息 |
|------|------|------|
| 订阅 | `/motion/plan_request` | `agx_motion_msgs/PlanRequest` |
| 订阅 | `/grasp/selected_pose` | `geometry_msgs/PoseStamped` |
| 订阅 | `/joint/states` | `sensor_msgs/JointState` |
| 订阅 | `/motion/execute_result` | `agx_motion_msgs/ExecuteResult` |
| 发布 | `/motion/trajectory` | `agx_motion_msgs/MotionTrajectory` |
| 发布 | `/motion/execute_feedback` | `agx_motion_msgs/ExecuteFeedback` |

## 3. PlanRequest 规划类型

| `plan_type` | 常量 | 必填字段 |
|-------------|------|----------|
| 0 | `PLAN_TYPE_JOINT` | `joint_goal` |
| 1 | `PLAN_TYPE_NAMED_TARGET` | `named_target`（如 `grasp_ready`、`home`） |
| 2 | `PLAN_TYPE_POSE` | `pose_goal` 或 `/grasp/selected_pose` |
| 3 | `PLAN_TYPE_CARTESIAN` | `cartesian_direction`、`cartesian_max_dist` |

`execute=true` 时：规划成功后发布轨迹，并阻塞等待 `arm_controller_node` 回传 `ExecuteResult`。

## 4. 参数

见 `config/motion_planner_params.yaml`。

## 5. 启动

```bash
# 仿真（含 MoveIt + ros2_control）
ros2 launch agx_motion_planner motion_stack.launch.py follow:=false

# 真机（无 ros2_control，推荐）
ros2 launch agx_motion_planner motion_stack_real.launch.py arm_type:=piper_l effector_type:=agx_gripper

# 等价写法
ros2 launch agx_motion_planner motion_stack.launch.py follow:=true use_sim:=false
```

## 6. 依赖

- `agx_arm_moveit`（move_group）
- `agx_motion_msgs`

## 7. 扩展建议

- 增加 `attach_object` / `detach_object` 服务，供 FSM 更新规划场景
- 对接 `/grasp/selected_pose` 与手眼标定结果，实现感知驱动抓取
