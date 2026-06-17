# Jetson RS232 ROS2 集成设计

| 元数据 | 值 |
|--------|-----|
| **文档版本** | v1.7 |
| **日期** | 2026-06-15 |
| **状态** | 阶段 6 RS232 + agv_base_driver **联调通过**；**时间同步 START/PING/QUERY 已通过**（见 [JETSON时间同步联调.md](./JETSON时间同步联调.md)） |
| **工作区** | `~/catkin_ws/cangyirobot`（**不是** `~/catkin_ws`） |
| **关联协议** | [PROTOCOL_V3.md](./PROTOCOL_V3.md) |
| **关联文档** | [JETSON_CAN_ROS2集成设计.md](./JETSON_CAN_ROS2集成设计.md)、[JETSON时间同步联调.md](./JETSON时间同步联调.md) |
| **联调工具** | `tools/jetson_rs232_link_test.py`、`tools/jetson_time_ping_test.py` |

本文档定义 Jetson 与 STM32B 之间 **RS232（USART2 PA2/PA3）链路** 在 ROS2 侧的架构：如何把 V3 24 字节帧解析并 **分发到标准 Topic**，并与团队节点分层、`agv_base_driver_node` 对接。

> **与 CAN 的关系**：V3 帧（type 0x01/0x02/0x03）字段与 `Jetson_CAN协议.md` 一致。**GPS、时间同步、故障、状态查询** 等服务载荷与 CAN 0x104~0x10B **相同**，RS232 上经 **0xA5 服务帧（11B）** 封装传输（见 §3.2）。

---

## 0. 环境与编译（每个新终端先执行）

代码已迁入 **`~/catkin_ws/cangyirobot`**。旧的 `~/catkin_ws/install/` **已删除**，不要再 `source` 它。

```bash
# 每个新终端
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash

# 若提示 install 不存在，先编译一次
cd ~/catkin_ws/cangyirobot
colcon build --packages-select jetson_can_msgs scr_sensor rs232_gateway --symlink-install
source install/setup.bash
```

**检查环境是否正常：**

```bash
# 不应报错
ros2 pkg list | grep rs232_gateway

# 串口设备（STM32B 多为 Prolific → ttyUSB7，插拔后可能变化）
ls -l /dev/serial/by-id/ | grep -i prolific

# 是否被占用（有输出则先 pkill）
fuser -v /dev/ttyUSB6 2>/dev/null
pkill -f jetson_bridge    # 旧 bridge 占口时
pkill -f rs232_gateway    # 重启 gateway 前
```

**`.bashrc` 修正**：若开终端仍报 `catkin_ws/install/setup.bash: No such file`，把其中路径改为：

```bash
source ~/catkin_ws/cangyirobot/install/setup.bash
```

---

## 1. 背景与目标

### 1.1 硬件链路

| 项目 | 说明 |
|------|------|
| 物理接口 | USB-TTL（本机多为 **Prolific**）↔ STM32B **USART2** |
| STM32 引脚 | **PA2=TX → Jetson RX**，**PA3=RX ← Jetson TX**，**GND 共地** |
| 波特率 | **115200 8N1** |
| 调试口 | 板载 **USART1 USB**，看 `[JETSON CMD]` / `ARB=` |

### 1.2 链路验证（阶段 1 测试命令）

**测试前**：确认 F407 调试口有 `[JETSON] USART2 PA2/PA3, 115200, 24-byte V3 frame`；KEY0 非 FORCE STOP。

```bash
cd ~/catkin_ws/cangyirobot
PORT=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0

# 步骤 A：只收不发（验上行）
python3 tools/jetson_rs232_link_test.py --port "$PORT" --listen-only --time 5
# 通过：统计里 0x02 > 0，0x03 > 0

# 步骤 B：20ms 发心跳 v=0 + 收上行（验双向）
python3 tools/jetson_rs232_link_test.py --port "$PORT" --time 10
# 通过：✅ 步骤 B 通过；Jetson心跳=正常
# 同时看 F407 USB：约每 0.5s [JETSON CMD] seq=… v=0 …
```

| 步骤 | 结果（2026-06-11） |
|------|---------------------|
| A 只收上行 | ✅ 通过 |
| B 发心跳 | ✅ 通过 |
| 仲裁 NORMAL | ⚠️ 长期 DEGRADED（外设/策略，**不影响链路 OK**） |

### 1.3 ROS2 目标

1. **`rs232_gateway_node`**：串口 ↔ `/jetson_rs232/*`（**已实现**）
2. **`agv_base_driver_node`**：融合 `/vehicle/vehicle_data`，处理 `/cmd_vel`（**已实现**）
3. **弃用** `ds_jetson_bridge` 单体节点（阶段 7）

---

## 2. 与 CAN 路径对比

（架构对照见 [JETSON_CAN_ROS2集成设计.md](./JETSON_CAN_ROS2集成设计.md)）

```text
RS232 → rs232_gateway → /jetson_rs232/*
CAN2  → can_gateway   → /jetson_can/*
              └─→ agv_base_driver → /vehicle/vehicle_data, /odom
```

**从 `agv_base_driver` 起的端到端信息流见 §3.1。**

---

## 3. 节点分层

| 节点 | 职责 | 测试命令 |
|------|------|----------|
| **rs232_gateway** | 串口 + 解析 + Topic | 见 §4、§7 |
| **agv_base_driver** | `/cmd_vel` + VehicleData + odom | 见 §8 |
| ds_jetson_bridge（旧） | 勿与 gateway 同时占口 | `pkill -f jetson_bridge` |

### 3.1 端到端信息流（从 `agv_base_driver` 起）

> 本节是 **ROS 侧完整数据流**；§9.2 记录联调中遇到的 **Jetson 软件问题** 与 **F407 外设问题** 的分别。

```text
                    ┌─────────────────────────────────────────────────────────┐
                    │  上层（nav_manager / teleop / 测试 pub）              │
                    │  发布 geometry_msgs/Twist → /cmd_vel                │
                    └──────────────────────────┬──────────────────────────┘
                                               │
                    ┌──────────────────────────▼──────────────────────────┐
                    │  agv_base_driver_node                              │
                    │  · 订阅 /cmd_vel → twist_to_motion()               │
                    │  · 50Hz 发布 jetson_can_msgs/V3Command              │
                    │      → /jetson_rs232/command                      │
                    │  · 订阅 /jetson_rs232/v3_status (0x02)            │
                    │         /jetson_rs232/v3_ext_status (0x03)          │
                    │  · 融合 → scr_sensor/VehicleData                    │
                    │      → /vehicle/vehicle_data                     │
                    │  · 底盘反馈积分 → nav_msgs/Odometry → /odom         │
                    │  · TF: odom → base_link                          │
                    └──────────────┬────────────────────▲─────────────────┘
                                   │                    │
                    ┌──────────────▼────────────────────┴─────────────────┐
                    │  rs232_gateway_node                                │
                    │  · 订阅 /jetson_rs232/command → 编码 V3 0x01       │
                    │  · 50Hz UART TX（PA2/PA3, 115200 8N1）           │
                    │  · UART RX 解析 0x02/0x03 + 0xA5 服务帧(GPS等)      │
                    │  · 发布 /jetson_rs232/link (Bool)                  │
                    └──────────────┬────────────────────▲─────────────────┘
                                   │ RS232 24B V3       │
                    ┌──────────────▼────────────────────┴─────────────────┐
                    │  STM32B (F407)                                    │
                    │  · 收 0x01 → [JETSON CMD] v/ω/steer             │
                    │  · Arbiter 仲裁（NORMAL / SPEED_LIMIT / …）         │
                    │  · CAN 0x111 等 → Ranger 底盘                     │
                    │  · CAN 0x221 等回馈 → 组 0x02/0x03 上行          │
                    └───────────────────────────────────────────────────┘
```

**Topic 速查（agv_base_driver 相关）：**

| 方向 | Topic | 消息 | 节点 |
|------|-------|------|------|
| 入 | `/cmd_vel` | geometry_msgs/Twist | ← 上层 |
| 入 | `/jetson_rs232/v3_status` | jetson_can_msgs/V3Status | ← rs232_gateway |
| 入 | `/jetson_rs232/v3_ext_status` | jetson_can_msgs/V3ExtStatus | ← rs232_gateway |
| 出 | `/jetson_rs232/command` | jetson_can_msgs/V3Command | → rs232_gateway |
| 出 | `/vehicle/vehicle_data` | scr_sensor/VehicleData | → 导航/监控 |
| 出 | `/odom` | nav_msgs/Odometry | → 导航/定位 |
| 出 | TF `odom`→`base_link` | geometry_msgs/TransformStamped | → tf2 |

**速度单位换算（与 V3 协议一致）：**

| ROS `/cmd_vel` | V3Command / V3Status |
|---------------|---------------------|
| `linear.x` m/s | `v_mm_s` = x × 1000 |
| `angular.z` rad/s | `omega_millirad_s` = z × 1000 |
| `linear.y`（横移） | `twist_to_motion()` → steer + motion_model |

**联调成功判据（F407 调试口 + ROS 两侧）：**

| 层级 | 期望 |
|------|------|
| ROS | `cmd_vel → v=200mm/s` 日志；`/jetson_rs232/command` 的 `v_mm_s` 非 0 |
| F407 收令 | `[JETSON CMD] v=200 ...` |
| F407 仲裁 | `[CMDOUT] v=200 ... mode=NORMAL`（**非 EMERGENCY**） |
| 底盘 | `[MOTION] FB` 有非零速度；`/vehicle/vehicle_data` 的 `fb_v_mm_s` 跟随 |

---

**启动 gateway（终端 1）：**

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash
ros2 launch rs232_gateway rs232_gateway.launch.py
# 期望日志：串口已连接 / 链路: 已连接
```

**验证节点（终端 2）：**

```bash
source ~/catkin_ws/cangyirobot/install/setup.bash
ros2 node list | grep rs232_gateway
```

---

## 4. 消息包

### 4.1 复用 `jetson_can_msgs`

| 消息 | RS232 | Topic |
|------|-------|-------|
| `V3Status.msg` | 0x02 | `/jetson_rs232/v3_status` |
| `V3ExtStatus.msg` | 0x03 | `/jetson_rs232/v3_ext_status` |
| `V3Command.msg` | 0x01 | `/jetson_rs232/command` |

**编译消息包：**

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_can_msgs scr_sensor --symlink-install
source install/setup.bash

# 查看消息定义
ros2 interface show jetson_can_msgs/msg/V3Status
ros2 interface show jetson_can_msgs/msg/V3ExtStatus
ros2 interface show scr_sensor/msg/VehicleData
```

### 4.2 gateway Topic 联调（阶段 4 核心测试）

gateway 运行中，在**另一终端**执行：

```bash
source ~/catkin_ws/cangyirobot/install/setup.bash

# 4.1 状态帧 0x02
ros2 topic echo /jetson_rs232/v3_status --once
# 期望：safety_state, link_state, fb_v_mm_s, sonar_front_mm 等

# 4.2 扩展帧 0x03
ros2 topic echo /jetson_rs232/v3_ext_status --once
# 期望：wheel_rf, steer_rf_millirad 等

# 4.3 发布频率
ros2 topic hz /jetson_rs232/v3_status
ros2 topic hz /jetson_rs232/v3_ext_status
# 期望：v3_status 约 25~50 Hz

# 4.4 链路状态
ros2 topic echo /jetson_rs232/link --once
# 期望：data: true

# 4.5 列出所有 jetson_rs232 topic
ros2 topic list | grep jetson_rs232
```

**可选：原始 24B 调试**

```bash
ros2 launch rs232_gateway rs232_gateway.launch.py publish_raw:=true
ros2 topic echo /jetson_rs232/raw_rx --once
```

---

## 5. Topic 命名定稿

| Topic | 消息 | 发布者 |
|-------|------|--------|
| `/jetson_rs232/v3_status` | jetson_can_msgs/V3Status | rs232_gateway |
| `/jetson_rs232/v3_ext_status` | jetson_can_msgs/V3ExtStatus | rs232_gateway |
| `/jetson_rs232/command` | jetson_can_msgs/V3Command | agv_base_driver（gateway 订阅） |
| `/jetson_rs232/gps/a` | jetson_can_msgs/GpsFrameA | rs232_gateway（0xA5→0x104） |
| `/jetson_rs232/gps/b` | jetson_can_msgs/GpsFrameB | rs232_gateway（0xA5→0x105） |
| `/jetson_rs232/gps/c` | jetson_can_msgs/GpsFrameC | rs232_gateway（0xA5→0x106） |
| `/jetson_rs232/time_sync` | jetson_can_msgs/TimeSyncResponse | rs232_gateway（0xA5→0x108） |
| `/jetson_rs232/fault` | jetson_can_msgs/FaultReport | rs232_gateway（0xA5→0x109） |
| `/jetson_rs232/status_snapshot` | jetson_can_msgs/StatusSnapshot | rs232_gateway（0xA5→0x10B） |
| `/jetson_rs232/link` | std_msgs/Bool | rs232_gateway |
| `/vehicle/vehicle_data` | scr_sensor/VehicleData | agv_base_driver |
| `/odom` | nav_msgs/Odometry | agv_base_driver |
| `/cmd_vel` | geometry_msgs/Twist | nav_manager → agv_base_driver |

**command 下行自测（阶段 5，gateway 需在跑）：**

```bash
source ~/catkin_ws/cangyirobot/install/setup.bash

# 确认 gateway 在订阅
ros2 topic info /jetson_rs232/command -v

# 发零速 command（保活/测试）
ros2 topic pub /jetson_rs232/command jetson_can_msgs/msg/V3Command \
  "{header: {frame_id: 'test'}, seq: 0, mode_req: 1, v_mm_s: 0, \
    omega_millirad_s: 0, steer_millirad: 0, motion_model: 0}" -r 20

# F407 调试口应持续 [JETSON CMD] seq=… v=0 …
```

**断链测试（阶段 5 可选）：**

```bash
# 停发 command（Ctrl+C pub），或 pkill gateway，等 0.5s
# F407 应出现 Heartbeat lost / ARB=DEGRADED
```

---

## 6. 传输层：V3 + 服务帧

RS232 字节流上 **混传** 两类帧（靠首字节魔数区分）：

| 魔数 | 长度 | 用途 | rs232_gateway 发布 |
|------|------|------|-------------------|
| `0xAA` | 24 B | V3 控制/状态/扩展（0x01/0x02/0x03） | `v3_status`, `v3_ext_status` |
| `0xA5` | 11 B | 服务帧：CAN ID + 8B 载荷 | `gps/*`, `time_sync`, `fault`, … |

服务帧格式：`[0xA5][ID_H][ID_L][8B PAYLOAD]`，载荷定义与 [Jetson_CAN协议.md](./Jetson_CAN协议.md) §7~§8 **完全相同**。

**GPS 验证（gateway 运行中）：**

```bash
ros2 topic hz /jetson_rs232/gps/a    # 约 10 Hz（有 fix 时）
ros2 topic echo /jetson_rs232/gps/b --once   # lat_e7
ros2 topic echo /jetson_rs232/gps/c --once   # lon_e7, alt_dm
# flags bit0(POS_VALID) 见 GpsFrameA；无 fix 时 lat/lon 可能无效
```

**IMU+GPS 融合**：

```bash
# 仅 GPS → /fix（需 rs232_gateway 在跑）
ros2 launch gps_rs232_to_fix gps_rs232_to_fix.launch.py

# 底盘 + STM32 GPS + USB IMU + 融合（室外有 fix 时）
ros2 launch gps_rs232_to_fix jetson_rs232_gps_imu_fusion.launch.py imu_port:=/dev/imu_usb
ros2 topic echo /fix --once
ros2 topic echo /fused_path --once
```

节点：`gps_rs232_to_fix`（`src/drivers/gps_rs232_to_fix/`），订阅 `/jetson_rs232/gps/{a,b,c}`，发布 `/fix`。

---

## 7. V3 协议要点（0xAA 帧）

| type | 方向 | gateway |
|------|------|---------|
| 0x01 | Jetson→STM32 | 订阅 command，50Hz 下发 |
| 0x02 | STM32→Jetson | → V3Status |
| 0x03 | STM32→Jetson | → V3ExtStatus |

协议细节：[PROTOCOL_V3.md](./PROTOCOL_V3.md)

**无 ROS 快速验帧（与 §1.2 相同）：**

```bash
cd ~/catkin_ws/cangyirobot
python3 tools/jetson_rs232_link_test.py --time 5
```

---

## 8. `rs232_gateway` 包

**路径**：`src/drivers/rs232_gateway/`

**编译：**

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
colcon build --packages-select rs232_gateway ds_jetson_bridge --symlink-install
source install/setup.bash
```

**launch 参数示例：**

```bash
ros2 launch rs232_gateway rs232_gateway.launch.py \
  serial_port:=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0 \
  tx_rate_hz:=50.0 \
  publish_raw:=false
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `serial_port` | Prolific by-id | STM32B 口 |
| `baud_rate` | 115200 | 8N1 |
| `tx_rate_hz` | 50 | 心跳/下发周期 |
| `auto_reconnect` | true | USB 热插拔 |
| `time_sync_enable` | true | START / 1Hz PING / 10s QUERY（见 [JETSON时间同步联调.md](./JETSON时间同步联调.md)） |
| `time_sync_ping_interval_s` | 1.0 | PING 间隔 |
| `time_sync_query_interval_s` | 10.0 | QUERY 间隔 |

**解析能力**：V3 0x02/0x03 + 服务帧 0xA5（GPS 0x104~106、0x108~0x10B）。

**时间同步联调**（不依赖 ROS）：

```bash
python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --probe
python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --query-only
python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --count 20 --query
```

详见 **[JETSON时间同步联调.md](./JETSON时间同步联调.md)**。

---

## 9. `agv_base_driver` 包

**路径**：`src/drivers/agv_base_driver/`

**职责**：

- 订阅 `/cmd_vel` → 发布 `/jetson_rs232/command`（50Hz，`twist_to_motion` 映射）
- 订阅 `/jetson_rs232/v3_status`、`v3_ext_status` → 融合发布 `/vehicle/vehicle_data`
- 由底盘反馈积分发布 `/odom` 与 `odom → base_link` TF
- `cmd_vel` 超时 1s 零速恢复（逻辑同旧 `jetson_bridge_node`）

**编译：**

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
colcon build --packages-select agv_base_driver --symlink-install
source install/setup.bash
```

**一键 bringup（gateway + driver）：**

```bash
pkill -f jetson_bridge; pkill -f rs232_gateway; pkill -f agv_base_driver
ros2 launch agv_base_driver jetson_rs232_bringup.launch.py
```

**仅 driver（gateway 已在跑）：**

```bash
ros2 launch agv_base_driver agv_base_driver.launch.py link_type:=rs232
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `link_type` | rs232 | Topic 前缀 `/jetson_{link_type}/` |
| `cmd_vel_topic` | /cmd_vel | 速度指令输入 |
| `cmd_rate_hz` | 50 | command 发布频率 |
| `cmd_timeout_ms` | 200 | cmd_vel 超时（bringup 默认 2000） |
| `cruise_scale` | 1.0 | 速度缩放 |
| `max_linear_m_s` | 0.8 | 线速度限幅 |
| `max_angular_rad_s` | 1.0 | 角速度限幅 |

**阶段 6 联调命令：**

```bash
source ~/catkin_ws/cangyirobot/install/setup.bash

# 终端 1：bringup
ros2 launch agv_base_driver jetson_rs232_bringup.launch.py

# 终端 2：看融合状态
ros2 topic hz /vehicle/vehicle_data
ros2 topic echo /vehicle/vehicle_data --once

# 终端 3：速度测试（x=0.2 m/s；超过 max_linear_m_s=0.8 会被限幅）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 20

# 若 driver 收不到 cmd_vel，检查 QoS（ros2 topic pub 默认 durability=transient_local）
ros2 topic echo /jetson_rs232/command --field v_mm_s
# 有非零 v_mm_s 即链路 OK；仍为 0 时可加：--qos-durability volatile

# 确认 command 由 driver 发布
ros2 topic info /jetson_rs232/command -v
```

---

## 10. 开发任务清单（按顺序做）

> **当前**：阶段 7 弃用旧 bridge

| 阶段 | 内容 | 状态 | 测试入口 |
|------|------|------|----------|
| 1 | 链路 A+B | ✅ | §1.2 |
| 2 | 消息包编译 | ✅ | §4.1 |
| 3 | rs232_gateway 实现 | ✅ | §7 编译 |
| 4 | gateway ROS 联调 | ✅ | §4.2 |
| 5 | command 下行 | ✅ | §5 / §8 bringup |
| 6 | agv_base_driver + 实车运动 | ✅ | §8、§9.2 |
| 7 | 弃用旧 bridge | ⬜ | §9.1 |

### 9.2 阶段 6 联调问题总结（2026-06-11）

本节汇总 RS232 新栈（`rs232_gateway` + `agv_base_driver`）联调过程中 **实际遇到** 的问题、根因与处理。**Jetson ROS 侧问题已修复；F407 外设问题需硬件/固件侧处理。**

#### A. Jetson / ROS 侧（软件）

| # | 现象 | 根因 | 处理 |
|---|------|------|------|
| A1 | 终端报 `catkin_ws/install/setup.bash: No such file` | 工作区已迁到 `cangyirobot`，旧路径失效 | `source ~/catkin_ws/cangyirobot/install/setup.bash`（§0） |
| A2 | `ros2 topic pub /cmd_vel` 在发，driver 报「cmd_vel 超时」 | `ros2 topic pub` 默认 QoS **durability=transient_local**，与 driver 原 **volatile** 订阅不兼容，消息到不了 | **已修**：`agv_base_driver` 的 `/cmd_vel` 订阅改为 **TRANSIENT_LOCAL**（兼容 pub 默认与 nav2）；临时可手加 `--qos-durability volatile` |
| A3 | `/jetson_rs232/command` 的 `v_mm_s` 一直为 0 | 同 A2，或处于 1s recovery | 先 `ros2 topic echo /jetson_rs232/command --field v_mm_s`；driver 日志应出现 `cmd_vel → v=…mm/s` |
| A4 | 发 `linear.x: 2.0` 但 command 最大约 800 | `max_linear_m_s` 默认 **0.8** m/s 限幅 | 正常；调参 `max_linear_m_s` 或发 ≤0.8 |
| A5 | gateway 与旧 bridge 冲突 | 同占 Prolific 串口 | 启动前 `pkill -f jetson_bridge` |
| A6 | launch 文件名写错 | `jetson_rs232.bringup`（点号）不存在 | 正确名：`jetson_rs232_bringup.launch.py` |

#### B. F407 / 外设侧（硬件与仲裁，ROS 无法绕过）

| # | 现象 | 根因 | 处理 |
|---|------|------|------|
| B1 | `[JETSON CMD] v=200` 有，但 `[CMDOUT] v=0 mode=EMERGENCY` | F407 **仲裁 EMERGENCY** 强制零速，与 Jetson 指令无关 | 查 F407 调试口 `ARB=`；ROS 链路此时 **已 OK** |
| B2 | 上电约 623ms：`NORMAL→SPEED_LIMIT→EMERGENCY` | 启动即进紧急态，常见于外设异常 | 见 B3/B4 |
| B3 | 四向超声 `F=--- B=--- L=--- R=---`；反复 `[DS] RX timeout, reset` | **超声 DS 模块通信失败**，Dist ctrl 开着时易 EMERGENCY | 查 IF1~4 接线与模块上电；`/vehicle/vehicle_data` 中 `sonar_*=65535` |
| B4 | `LF=0 RF=0 … v221=0` 长期为 0 | **底盘 CAN 无 0x221 等回馈**（Ranger 未上电或 CAN 未通） | 确认底盘上电、PA11/12 CAN H/L |
| B5 | 按 KEY1 只出现 `[BEEP] Distance alert ON/OFF` | 该固件版本 KEY1 可能 **只控蜂鸣**，不一定关 Dist ctrl | 以固件说明为准；外设正常后仲裁才会回到 NORMAL |
| B6 | `/vehicle/vehicle_data` 的 `safety_state=4` | 对应 EMERGENCY | 外设修复后应变 1~3；见 [README.md](./README.md) 仲裁表 |

#### C. 问题分层速查（一眼区分在哪一段）

| Jetson / ROS | F407 调试口 | 结论 |
|-------------|-------------|------|
| `/cmd_vel` 无 echo / driver 超时 | 无 `[JETSON CMD]` | **下行未到 F407** 或 QoS（A2） |
| `v_mm_s=200`，driver 正常 | 有 `[JETSON CMD] v=200`，`CMDOUT v=0 EMERGENCY` | **ROS OK，F407 仲裁/外设**（B1~B6） |
| 同上 | `[CMDOUT] v=200 NORMAL`，车仍不动 | **CAN / Ranger 侧**（B4） |

#### D. 本阶段结论

- **RS232 ROS 集成（gateway + agv_base_driver）联调通过**：`/cmd_vel` → command → F407 `[JETSON CMD]` 全链路正常。
- **实车能否动** 取决于 F407 仲裁是否放行（超声、CAN、按键等），不属于 Jetson driver 代码缺陷。
- 详细 Topic 命名另见 [TOPIC_NAMING.md](./TOPIC_NAMING.md)。

### 9.1 阶段 7 收尾

```bash
# 统一用新栈，不再启动旧 bridge
pkill -f jetson_bridge
ros2 launch agv_base_driver jetson_rs232_bringup.launch.py
```

---

## 11. 常见问题

| 现象 | 处理 |
|------|------|
| `catkin_ws/install/setup.bash: No such file` | 改用 `~/catkin_ws/cangyirobot/install/setup.bash`（§0） |
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `package 'rs232_gateway' not found` | §0 编译 + source |
| 串口打不开 / Errno 16 | `pkill -f jetson_bridge; pkill -f rs232_gateway` |
| 无 v3_status 数据 | 查接线、F407 是否 RS232 模式、Prolific 口是否正确 |
| pub cmd_vel 但 driver 报超时 / v_mm_s=0 | QoS 不匹配（§9.2 A2）；`ros2 topic echo /jetson_rs232/command --field v_mm_s` |
| `[JETSON CMD] v=200` 但车不动 | F407 `ARB=EMERGENCY`（§9.2 B1~B6）；非 ROS 问题 |
| DEGRADED / EMERGENCY 长期不变 | 超声 DS 超时、CAN 无反馈、Dist ctrl；见 §9.2 B3/B4 |

---

## 12. 相关文档

| 文档 | 说明 |
|------|------|
| [JETSON_CAN_ROS2集成设计.md](./JETSON_CAN_ROS2集成设计.md) | CAN 对称设计 |
| [JETSON时间同步联调.md](./JETSON时间同步联调.md) | START/PING/QUERY、offset/RTT、联调步骤 |
| [PROTOCOL_V3.md](./PROTOCOL_V3.md) | V3 24B 字段 |
| [README.md](./README.md) | 串口接线、旧 bridge 实操 |

---

## 13. 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-06-11 | 初稿 |
| v1.1 | 2026-06-11 | 实现 rs232_gateway；任务清单 |
| v1.2 | 2026-06-11 | §0 工作区说明；各章嵌入测试命令；修正 source 路径 |
| v1.3 | 2026-06-11 | 实现 agv_base_driver；§8 bringup 与阶段 6 测试命令 |
| v1.4 | 2026-06-11 | §3.1 端到端信息流；§9.2 联调问题总结；阶段 6 实车通过 |
| v1.5 | 2026-06-11 | RS232 v1.1 服务帧 0xA5；gateway 解析 GPS 0x104~106 |
| v1.6 | 2026-06-11 | `gps_rs232_to_fix`：GpsFrame → `/fix`；融合 bringup launch |
| v1.7 | 2026-06-15 | `time_sync.py` START/PING/QUERY；[JETSON时间同步联调.md](./JETSON时间同步联调.md) |
