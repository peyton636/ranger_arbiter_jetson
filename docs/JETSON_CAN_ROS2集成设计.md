# Jetson CAN ROS2 集成设计

| 元数据 | 值 |
|--------|-----|
| **文档版本** | v1.0（定稿） |
| **日期** | 2026-06-10 |
| **状态** | 接口层已定稿；`can_gateway_node` / `agv_base_driver_node` 待实现 |
| **关联协议** | [Jetson_CAN协议.md](./Jetson_CAN协议.md) v1.4 |
| **关联分支** | `cangyirobot` → `feature/integrate-ds-packages` |

本文档汇总 Jetson 与 STM32B 之间 **CAN 链路在 ROS2 侧的架构、目录、消息、Topic 命名与开发计划**，作为团队联调与代码实现的统一参考。

---

## 1. 背景与目标

### 1.1 硬件链路

- **物理总线**：Jetson 专用 CAN2（与底盘 CAN1 隔离），默认 **500 kbps**，Classic CAN 2.0B
- **对端**：STM32B（麒麟 F407 仲裁板）
- **应用层**：V3 24 字节帧（Header + Type + Seq + Payload + XOR），详见协议文档

STM32 侧可通过宏 `JETSON_LINK_CAN` 在 **CAN2** 与 **USART2 串口** 之间切换。Jetson ROS2 侧对应两条路径：

| STM32 链路 | Jetson ROS2 包 | 状态 |
|------------|----------------|------|
| CAN2（目标路径） | `can_gateway`（待建） | 本文档设计对象 |
| USART2（备用） | `rs232_gateway`（待建，见 RS232 集成设计） | 串口 V3 桥 |

### 1.2 ROS2 目标

1. **can_gateway_node**：SocketCAN 读写、V3 组帧、GPS/故障/时间同步解析，发布标准 Topic
2. **agv_base_driver_node**：订阅解析结果，融合为车体业务状态，处理 `/cmd_vel`，发布 `/vehicle/vehicle_data` 与 `/odom`
3. **与底盘驱动解耦**：CAN 硬件与协议解析集中在 gateway，底盘逻辑在 agv_base_driver

---

## 2. 代码仓库与工作区

### 2.1 统一工作区：`cangyirobot`

原 `~/catkin_ws/src` 下的 `ds_*` 包已迁入 **`cangyirobot`**，后续以该仓库为唯一开发入口：

```
~/catkin_ws/cangyirobot/
├── src/
│   ├── drivers/              # 驱动层
│   │   ├── ds_jetson_bridge      # 串口 V3 备用
│   │   ├── ds_can_monitor        # CAN 调试（0x110/0x111 测距板）
│   │   ├── ds_serial_monitor
│   │   ├── ds_gps_driver
│   │   ├── ds_imu_driver
│   │   ├── can_gateway/          # 【待建】CAN 网关节点
│   │   ├── agv_base_driver/      # 【待建】底盘驱动节点
│   │   ├── ranger_ros2/          # gitignore，本地 clone
│   │   └── ugv_sdk/              # gitignore，本地 clone
│   ├── navigation/
│   │   ├── ds_gps_goal
│   │   └── ds_imu_gps_localization
│   ├── interfaces/
│   │   ├── jetson_can_msgs/      # Jetson CAN 解析后语义消息
│   │   └── scr_sensor/           # 车体业务层消息（VehicleData 等）
│   └── （文档见仓库根目录 docs/）
├── tools/GPS.py
├── build/ install/ log/          # colcon 产物，可删，不进 git
└── ...
```

### 2.2 编译

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash

# 接口包
colcon build --packages-select jetson_can_msgs scr_sensor --symlink-install

# 后续 gateway / driver 就绪后
colcon build --symlink-install

source install/setup.bash
```

> **说明**：`src/` 是**源代码目录**（必须保留）；`build/`、`install/`、`log/` 是编译产物（已在 `.gitignore`，可随时删除）。

### 2.3 Git 分支

- 集成分支：`feature/integrate-ds-packages`（从 `develop` 切出）
- 含：包迁移、`jetson_can_msgs`、`scr_sensor/VehicleData`、本文档

---

## 3. 节点分层（对齐团队架构表）

### 3.1 驱动和中间件层

| 节点 | 输入 | 输出 | 职责 |
|------|------|------|------|
| **can_gateway_node** | SocketCAN 硬件 | `/can/frame`、`/jetson_can/*` | CAN 收发、V3 组帧、协议解析、下行转发 |
| **agv_base_driver_node** | `/jetson_can/*`、`/cmd_vel` | `/vehicle/vehicle_data`、`/odom`、`/jetson_can/command` | 底盘运动控制、里程计、车体状态融合 |
| ds_jetson_bridge | 串口 | `stm32b/*`（旧路径） | 串口 V3 备用，CAN 稳定后可 deprecate |
| ds_can_monitor | SocketCAN | 终端显示 | **调试工具**，监听 0x110/0x111，非生产网关 |

### 3.2 职责边界

| 节点 | 做什么 | 不做什么 |
|------|--------|----------|
| **can_gateway_node** | SocketCAN；0x102/0x103 三片×8B 组 24B + XOR 校验；GPS 0x104~106 直解析；发原始帧与解析 Topic；订阅 `/jetson_can/command` 发 0x101 | 不算里程计；不处理 `/cmd_vel` 运动学 |
| **agv_base_driver_node** | 融合 V3Status + V3ExtStatus → VehicleData；`/cmd_vel` → V3Command；里程计 | 不直接 `socket(PF_CAN)` |

### 3.3 节点关系图

```text
                    ┌─────────────────────┐
  nav_manager ──────►│  agv_base_driver    │──────► /vehicle/vehicle_data
       │             │       _node         │──────► /odom
       │             └─────────┬───────────┘
       │                       │ /jetson_can/command
       │                       ▼
       │             ┌─────────────────────┐
       └─ /cmd_vel ─►│   can_gateway       │◄──── SocketCAN can2
                     │       _node         │
                     └─────────┬───────────┘
                               │
              ┌────────────────┼────────────────┐
              ▼                ▼                ▼
        /can/frame    /jetson_can/v3_*    /jetson_can/gps/*
        diagnostics     safety_monitor      融合 / tf
```

---

## 4. 消息包设计

### 4.1 三包分工（避免命名冲突）

| 包名 | 路径 | 用途 |
|------|------|------|
| **`jetson_can_msgs`** | `src/interfaces/jetson_can_msgs/` | Jetson↔STM32B **解析后**语义消息（V3、GPS、故障等） |
| **`can_msgs`**（ROS 系统 apt） | `apt install ros-humble-can-msgs` | **原始** SocketCAN 帧，用于 `/can/frame` |
| **`scr_sensor`** | `src/interfaces/scr_sensor/` | 车体/任务/安全 **业务层**消息 |

> **重要**：本项目语义包曾用名 `can_msgs`，已改名为 **`jetson_can_msgs`**，避免与 `ros-humble-can-msgs` 冲突。

### 4.2 jetson_can_msgs 消息列表

| 消息文件 | CAN ID | 方向 | 说明 |
|----------|--------|------|------|
| `V3Status.msg` | 0x102 | STM32→Jetson | V3 上行状态帧（type=0x02，24B 组帧后） |
| `V3ExtStatus.msg` | 0x103 | STM32→Jetson | V3 上行扩展帧（轮速、四轮转角、温度） |
| `GpsFrameA.msg` | 0x104 | STM32→Jetson | GPS 帧 A：flags、卫星数、HDOP、速度 |
| `GpsFrameB.msg` | 0x105 | STM32→Jetson | GPS 帧 B：纬度、航向 |
| `GpsFrameC.msg` | 0x106 | STM32→Jetson | GPS 帧 C：经度、海拔 |
| `TimeSyncResponse.msg` | 0x108 | STM32→Jetson | 时间同步响应（tick + Unix UTC） |
| `FaultReport.msg` | 0x109 | STM32→Jetson | 故障快照 |
| `StatusSnapshot.msg` | 0x10B | STM32→Jetson | 状态查询响应（8B 快照） |
| `V3Command.msg` | 0x101 | Jetson→STM32 | V3 下行控制帧（type=0x01） |

#### V3Status 主要字段（0x102）

| 字段 | 类型 | 说明 |
|------|------|------|
| `seq` | uint8 | STM32 发送序号 |
| `safety_state` | uint8 | 1=NORMAL, 2=SPEED_LIMIT, 3=DEGRADED, 4=EMERGENCY |
| `link_state` | uint8 | bit0 JETSON_HB_LOST, bit1 CHASSIS_FAULT, bit2 BEEP_ACTIVE |
| `limit_factor` | uint8 | 限速比例 0~100 |
| `fb_v_mm_s` | int16 | 底盘线速度 mm/s |
| `fb_omega_millirad_s` | int16 | 自旋速度，0.001 rad/s |
| `fb_steer_millirad` | int16 | 朝向/内转角，0.001 rad |
| `sonar_*_mm` | uint16 | 四向超声 mm，`65535` 无效 |
| `battery_voltage_0p1v` | uint16 | 电池电压，0.1 V |
| `battery_soc` | uint8 | SOC % |

#### V3ExtStatus 主要字段（0x103）

| 字段 | 类型 | 说明 |
|------|------|------|
| `wheel_rf/rr/lr/lf` | int16 | 四轮轮速 |
| `steer_*_millirad` | int16 | 四轮转角 |
| `motor_temp_max_c` | int8 | 8 路电机最高温度 ℃ |
| `driver_state_or` | uint8 | 8 路驱动状态按位 OR |

### 4.3 scr_sensor / VehicleData（车体综合状态）

**Topic**：`/vehicle/vehicle_data`  
**发布者**：`agv_base_driver_node`  
**路径**：`src/interfaces/scr_sensor/msg/VehicleData.msg`

由 `agv_base_driver_node` 将 gateway 发布的 `V3Status` + `V3ExtStatus` 融合为业务层消息：

| VehicleData 字段 | 来源 |
|------------------|------|
| `linear_velocity_mm_s`, `angular_velocity_millirad_s`, `steer_millirad` | V3Status |
| `safety_state`, `limit_factor`, `link_state` | V3Status |
| `sonar_front/back/left/right_mm` | V3Status |
| `battery_voltage_0p1v`, `battery_soc_percent` | V3Status |
| `wheel_rf/rr/lr/lf`, `motor_temp_max_c`, `driver_state_or` | V3ExtStatus |
| `v3_status_seq`, `v3_ext_status_seq`, `data_valid` | agv_base_driver 融合时填充 |

---

## 5. Topic 命名定稿

### 5.1 命名原则

1. **原始 CAN**：`/can/frame` → 系统 `can_msgs/msg/Frame`
2. **Jetson 解析结果**：`/jetson_can/*` → `jetson_can_msgs/*`
3. **车体业务状态**：`/vehicle/*` → `scr_sensor/*`
4. **下行控制**：`agv_base_driver` 发 `/jetson_can/command` → `can_gateway` 订阅 → CAN 发 0x101

### 5.2 Topic 定稿表

| # | Topic | 消息类型 | 发布节点 | 订阅节点 | QoS / 频率 | 说明 |
|---|-------|----------|----------|----------|------------|------|
| 1 | `/can/frame` | `can_msgs/msg/Frame` | can_gateway_node | agv_base_driver_node, diagnostics_node | BestEffort, 20~100 Hz | 原始 CAN 帧镜像 |
| 2 | `/jetson_can/v3_status` | jetson_can_msgs/V3Status | can_gateway_node | agv_base_driver, diagnostics, safety_monitor | Reliable, ~50 Hz | 0x102 |
| 3 | `/jetson_can/v3_ext_status` | jetson_can_msgs/V3ExtStatus | can_gateway_node | agv_base_driver, diagnostics | Reliable, ~25 Hz | 0x103 |
| 4 | `/jetson_can/gps/a` | jetson_can_msgs/GpsFrameA | can_gateway_node | 融合节点, diagnostics | Reliable, ~10 Hz | 0x104 |
| 5 | `/jetson_can/gps/b` | jetson_can_msgs/GpsFrameB | can_gateway_node | 融合节点 | Reliable, ~10 Hz | 0x105 |
| 6 | `/jetson_can/gps/c` | jetson_can_msgs/GpsFrameC | can_gateway_node | 融合节点 | Reliable, ~10 Hz | 0x106 |
| 7 | `/jetson_can/time_sync` | jetson_can_msgs/TimeSyncResponse | can_gateway_node | tf_manager, GPS 融合 | Reliable, 事件+10s | 0x108 |
| 8 | `/jetson_can/fault` | jetson_can_msgs/FaultReport | can_gateway_node | diagnostics, safety_monitor, system_supervisor | Reliable, 事件+1Hz | 0x109 |
| 9 | `/jetson_can/status_snapshot` | jetson_can_msgs/StatusSnapshot | can_gateway_node | diagnostics | Reliable, 按需 | 0x10B |
| 10 | `/jetson_can/command` | jetson_can_msgs/V3Command | agv_base_driver_node | can_gateway_node | Reliable, 20~50 Hz | 0x101 下行 |
| 11 | `/jetson_can/time_sync_request` | std_msgs/Empty | 定时器 / gateway | can_gateway_node | Reliable, 10s | 0x107 请求 |
| 12 | `/jetson_can/status_query` | std_msgs/Empty | diagnostics | can_gateway_node | Reliable, 按需 | 0x10A 请求 |
| 13 | `/vehicle/vehicle_data` | scr_sensor/VehicleData | agv_base_driver_node | nav_manager, system_supervisor, diagnostics | Reliable, 20~50 Hz | **车体综合状态** |
| 14 | `/odom` | nav_msgs/Odometry | agv_base_driver_node | nav_manager, tf_manager | Reliable, 30~50 Hz | 里程计 |
| 15 | `/cmd_vel` | geometry_msgs/Twist | nav_manager_node | agv_base_driver_node | Reliable, 20~50 Hz | 速度指令 |

### 5.3 端到端数据流

```text
SocketCAN (can2, 500 kbps)
        │
        ▼
 can_gateway_node
   ├─ publish  /can/frame
   ├─ publish  /jetson_can/v3_status       (0x102, ~20ms)
   ├─ publish  /jetson_can/v3_ext_status   (0x103, ~40ms)
   ├─ publish  /jetson_can/gps/{a,b,c}     (0x104~106, ~100ms)
   ├─ publish  /jetson_can/time_sync       (0x108)
   ├─ publish  /jetson_can/fault           (0x109)
   └─ subscribe /jetson_can/command       → TX 0x101

 agv_base_driver_node
   ├─ subscribe /jetson_can/v3_status, /jetson_can/v3_ext_status
   ├─ subscribe /cmd_vel
   ├─ publish   /vehicle/vehicle_data
   ├─ publish   /odom
   └─ publish   /jetson_can/command
```

---

## 6. CAN 协议要点（gateway 实现参考）

> 完整定义见 [Jetson_CAN协议.md](./Jetson_CAN协议.md)

### 6.1 CAN ID 分配（已实现部分）

| CAN ID | 方向 | 内容 |
|--------|------|------|
| 0x101 | Jetson→STM32 | V3 下行控制（type=0x01） |
| 0x102 | STM32→Jetson | V3 上行状态（type=0x02） |
| 0x103 | STM32→Jetson | V3 上行扩展（type=0x03） |
| 0x104~0x106 | STM32→Jetson | GPS 三帧（独立 ID，无需组帧） |
| 0x107 | Jetson→STM32 | 时间同步请求 |
| 0x108 | STM32→Jetson | 时间同步响应 |
| 0x109 | STM32→Jetson | 故障上报 |
| 0x10A | Jetson→STM32 | 状态查询请求 |
| 0x10B | STM32→Jetson | 状态查询响应 |

### 6.2 V3 组帧（0x101 / 0x102 / 0x103）

Classic CAN 下 24B 应用层拆为 **同 ID 三帧 × 8B**，接收端按序拼接后再校验：

```text
frag0 = v3[0..7]
frag1 = v3[8..15]
frag2 = v3[16..23]
xor   = frame[0] ^ frame[1] ^ ... ^ frame[22]  == frame[23]
```

### 6.3 GPS 帧（0x104 / 0x105 / 0x106）

- 三个**独立 CAN ID**，各 8B，**无需缓存组帧**
- byte0 魔数 `0xA4`；不复用 V3 XOR
- 某一帧丢失不影响其余帧

### 6.4 可复用代码

| 现有文件 | 复用方式 |
|----------|----------|
| `ds_jetson_bridge/jetson_protocol.py` | 迁入 `can_gateway/jetson_can_protocol.py`：`parse_uplink_status()`、`encode_downlink()`、XOR、s16/u16 BE |
| `ds_can_monitor/can_monitor_node.py` | 参考 `CanSocketReader`（SocketCAN raw socket） |
| `ds_jetson_bridge` | 串口路径保留；CAN 上线后作 fallback |

---

## 7. can_gateway 包规划（待实现）

### 7.1 目录结构

```
src/drivers/can_gateway/
├── package.xml
├── setup.py
├── launch/can_gateway.launch.py
└── can_gateway/
    ├── can_gateway_node.py       # ROS 节点：pub/sub
    ├── socketcan_io.py           # SocketCAN 读写
    ├── v3_reassembler.py         # 0x102/0x103 三片组帧 + XOR
    └── jetson_can_protocol.py    # 从 jetson_protocol.py 迁移
```

### 7.2 节点参数（建议默认值）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can2` | SocketCAN 网口名 |
| `v3_timeout_ms` | `300` | 组帧超时（与心跳一致） |
| `publish_raw_frames` | `true` | 是否发 `/can/frame` |
| `time_sync_period_s` | `10.0` | 0x107 周期请求 |

### 7.3 agv_base_driver 包规划（待实现）

```
src/drivers/agv_base_driver/
├── agv_base_driver_node.py       # 融合 + /cmd_vel + /odom
└── ...
```

---

## 8. 开发顺序

| 阶段 | 内容 | 状态 |
|------|------|------|
| 1 | 迁入 `ds_*` 包到 `cangyirobot` | ✅ 完成 |
| 2 | 定义 `jetson_can_msgs` + `scr_sensor/VehicleData` | ✅ 完成 |
| 3 | 本文档定稿 | ✅ 完成 |
| 4 | 实现 `can_gateway_node` | ⬜ 待做 |
| 5 | 实现 `agv_base_driver_node` | ⬜ 待做 |
| 6 | CAN 联调替换串口路径 | ⬜ 待做 |
| 7 | 补充 `scr_sensor/NavState` 等任务层消息 | ⬜ 按需 |

---

## 9. 依赖安装

```bash
# ROS2 原始 CAN 帧消息（/can/frame）
sudo apt install ros-humble-can-msgs

# 编译接口包
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_can_msgs scr_sensor --symlink-install
source install/setup.bash

# 验证
ros2 interface show jetson_can_msgs/msg/V3Status
ros2 interface show scr_sensor/msg/VehicleData
```

---

## 10. 相关文档索引

| 文档 | 说明 |
|------|------|
| [README.md](./README.md) | 串口联调、`ds_jetson_bridge`、GPS/IMU 实操 |
| [JETSON_RS232_ROS2集成设计.md](./JETSON_RS232_ROS2集成设计.md) | RS232 路径 Topic 分发与 gateway |
| [Jetson_CAN协议.md](./Jetson_CAN协议.md) | CAN2 完整协议 v1.4 |
| [PROTOCOL_V3.md](./PROTOCOL_V3.md) | V3 24B 应用层帧 |
| [STM32B_FIRMWARE_NOTES.md](./STM32B_FIRMWARE_NOTES.md) | STM32B 固件说明 |
| [TOPIC_NAMING.md](./TOPIC_NAMING.md) | Topic 命名速查（与本文 §5 一致） |

---

## 11. 变更记录

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| v1.0 | 2026-06-10 | 初稿：工作区迁移、节点分层、jetson_can_msgs 重命名、VehicleData、Topic 定稿、gateway 规划 |
