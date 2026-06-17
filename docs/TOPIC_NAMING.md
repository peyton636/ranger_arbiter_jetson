# Jetson CAN Topic 命名速查

> **完整架构说明**见 [JETSON_CAN_ROS2集成设计.md](./JETSON_CAN_ROS2集成设计.md)  
> RS232 路径见 [JETSON_RS232_ROS2集成设计.md](./JETSON_RS232_ROS2集成设计.md) §5

与团队节点分层表对齐：`can_gateway_node` 负责 CAN 硬件与协议解析；`agv_base_driver_node` 负责底盘逻辑与 `/vehicle/vehicle_data`。

## 命名原则

1. **原始 CAN**：`/can/frame`，使用系统包 `ros-humble-can-msgs` 的 `can_msgs/msg/Frame`（**不是** `jetson_can_msgs`）。
2. **Jetson 解析结果**：`/jetson_can/`*，使用 `jetson_can_msgs/*`。
3. **车体业务状态**：`/vehicle/`*，使用 `scr_sensor/*`，由 `agv_base_driver_node` 发布。
4. **下行控制**：`agv_base_driver_node` 发布 → `can_gateway_node` 订阅 → SocketCAN 发送。

---

## Topic 定稿表


| #   | Topic                           | 消息类型                                   | 发布节点                    | 订阅节点                                                                | QoS / 频率              | 说明                        |
| --- | ------------------------------- | -------------------------------------- | ----------------------- | ------------------------------------------------------------------- | --------------------- | ------------------------- |
| 1   | `/can/frame`                    | `can_msgs/msg/Frame`                   | `can_gateway_node`      | `agv_base_driver_node`, `diagnostics_node`                          | BestEffort, 20~100 Hz | 原始 CAN 帧镜像（联调/诊断）         |
| 2   | `/jetson_can/v3_status`         | `jetson_can_msgs/msg/V3Status`         | `can_gateway_node`      | `agv_base_driver_node`, `diagnostics_node`, `safety_monitor_node`   | Reliable, ~50 Hz      | 0x102 状态帧解析               |
| 3   | `/jetson_can/v3_ext_status`     | `jetson_can_msgs/msg/V3ExtStatus`      | `can_gateway_node`      | `agv_base_driver_node`, `diagnostics_node`                          | Reliable, ~25 Hz      | 0x103 扩展帧解析               |
| 4   | `/jetson_can/gps/a`             | `jetson_can_msgs/msg/GpsFrameA`        | `can_gateway_node`      | 融合节点, `diagnostics_node`                                            | Reliable, ~10 Hz      | 0x104 GPS 帧 A             |
| 5   | `/jetson_can/gps/b`             | `jetson_can_msgs/msg/GpsFrameB`        | `can_gateway_node`      | 融合节点                                                                | Reliable, ~10 Hz      | 0x105 GPS 帧 B             |
| 6   | `/jetson_can/gps/c`             | `jetson_can_msgs/msg/GpsFrameC`        | `can_gateway_node`      | 融合节点                                                                | Reliable, ~10 Hz      | 0x106 GPS 帧 C             |
| 7   | `/jetson_can/time_sync`         | `jetson_can_msgs/msg/TimeSyncResponse` | `can_gateway_node`      | `tf_manager_node`, GPS 融合节点                                         | Reliable, 事件 + 10 s   | 0x108 时间同步                |
| 8   | `/jetson_can/fault`             | `jetson_can_msgs/msg/FaultReport`      | `can_gateway_node`      | `diagnostics_node`, `safety_monitor_node`, `system_supervisor_node` | Reliable, 事件 + 1 Hz   | 0x109 故障上报                |
| 9   | `/jetson_can/status_snapshot`   | `jetson_can_msgs/msg/StatusSnapshot`   | `can_gateway_node`      | `diagnostics_node`                                                  | Reliable, 按需          | 0x10B 查询快照                |
| 10  | `/jetson_can/command`           | `jetson_can_msgs/msg/V3Command`        | `agv_base_driver_node`  | `can_gateway_node`                                                  | Reliable, 20~50 Hz    | 0x101 下行控制（gateway 发 CAN） |
| 11  | `/jetson_can/time_sync_request` | `std_msgs/msg/Empty`                   | `can_gateway_node` 或定时器 | `can_gateway_node`                                                  | Reliable, 10 s        | 0x107 请求（内部或对外）           |
| 12  | `/jetson_can/status_query`      | `std_msgs/msg/Empty`                   | `diagnostics_node`      | `can_gateway_node`                                                  | Reliable, 按需          | 0x10A 查询（内部或对外）           |
| 13  | `/vehicle/vehicle_data`         | `scr_sensor/msg/VehicleData`           | `agv_base_driver_node`  | `nav_manager_node`, `system_supervisor_node`, `diagnostics_node`    | Reliable, 20~50 Hz    | **车体综合状态（定稿）**            |
| 14  | `/odom`                         | `nav_msgs/msg/Odometry`                | `agv_base_driver_node`  | `nav_manager_node`, `tf_manager_node`                               | Reliable, 30~50 Hz    | 里程计（ROS 标准）               |
| 15  | `/cmd_vel`                      | `geometry_msgs/msg/Twist`              | `nav_manager_node`      | `agv_base_driver_node`                                              | Reliable, 20~50 Hz    | 速度指令（ROS 标准）              |


---

## 数据流

```text
SocketCAN (can2, 500 kbps)
        │
        ▼
 can_gateway_node
   ├─ publish /can/frame
   ├─ publish /jetson_can/v3_status      (0x102)
   ├─ publish /jetson_can/v3_ext_status  (0x103)
   ├─ publish /jetson_can/gps/{a,b,c}    (0x104~106)
   ├─ publish /jetson_can/time_sync      (0x108)
   ├─ publish /jetson_can/fault          (0x109)
   └─ subscribe /jetson_can/command      → TX 0x101

 agv_base_driver_node
   ├─ subscribe /jetson_can/v3_status, /jetson_can/v3_ext_status
   ├─ subscribe /cmd_vel
   ├─ publish   /vehicle/vehicle_data    (scr_sensor/VehicleData)
   ├─ publish   /odom
   └─ publish   /jetson_can/command
```

---

## 消息包对照


| 包名                 | 用途                          | 与官方包关系                            |
| ------------------ | --------------------------- | --------------------------------- |
| `jetson_can_msgs`  | Jetson↔STM32B **解析后**语义消息   | 避免与 `ros-humble-can-msgs` 冲突      |
| `can_msgs`（系统 apt） | `/can/frame` 原始 SocketCAN 帧 | `apt install ros-humble-can-msgs` |
| `scr_sensor`       | 车体/任务/安全 **业务层**消息          | `VehicleData` 等                   |


---

## VehicleData 字段来源


| VehicleData 字段                                                          | 来源                    |
| ----------------------------------------------------------------------- | --------------------- |
| `linear_velocity_mm_s`, `angular_velocity_millirad_s`, `steer_millirad` | V3Status              |
| `safety_state`, `limit_factor`, `link_state`                            | V3Status              |
| `sonar_*_mm`                                                            | V3Status              |
| `battery_`*                                                             | V3Status              |
| `wheel_*`, `motor_temp_max_c`, `driver_state_or`                        | V3ExtStatus           |
| `v3_status_seq`, `v3_ext_status_seq`, `data_valid`                      | agv_base_driver 融合时填充 |


---

## 变更记录


| 版本   | 日期         | 说明                                                                    |
| ---- | ---------- | --------------------------------------------------------------------- |
| v1.0 | 2026-06-10 | 初稿：`jetson_can_msgs` 重命名；`/vehicle/vehicle_data` + `/jetson_can/*` 定稿 |


