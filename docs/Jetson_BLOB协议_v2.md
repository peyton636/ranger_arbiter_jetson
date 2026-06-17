# Jetson 与 MCU BLOB 二进制协议（v2.0 草案）

| 元数据 | 值 |
|--------|-----|
| **协议版本** | v2.0-draft.5.5 |
| **文档日期** | 2026-06-15 |
| **物理层** | RS232 115200 8N1（USART2）或 CAN2 500 kbps（编译期二选一） |
| **编码** | 多字节整数 **大端 BE**；`#pragma pack(1)` |

> 横排表约定：与 `Jetson_RS232协议.md` 相同——首列留空；多字节范围写 `4-5`（ASCII 连字符）；分隔行 `|:---:|` 个数必须与列数一致。

**与 V3 的关系**：BLOB PAYLOAD（struct）为 v2 新定义；传输层沿用 V3 思路——同一条线格式，RS232 整帧发送，CAN 按 ID 8 字节顺序分片。时间同步等 `0xA5` / `0x107~0x108` 服务帧保持不变，与 BLOB 混传。

硬件切换见 `硬件连接与通信协议.md` §2.2（`JETSON_LINK_CAN` 1=CAN2，0=USART2）。

**Jetson ROS2 实现**：`rs232_gateway` 包，`use_blob_v2:=true`（默认）时发 `0xAB` MSG `0x01`，解析上行 BLOB 并映射到 `/jetson_rs232/v3_status` 等 Topic，与 `agv_base_driver` 兼容。

---

## 0. 传输封装（所有 BLOB 业务帧共用）

### 0.1 线格式（Wire Image）

RS232 与 CAN 共用同一字节序列：先 **9 字节头**，再 **LEN 字节 PAYLOAD**。

`[0xAB][VER=0x01][MSG_ID][SEQ][LEN_H][LEN_L][FRAG_IDX][FRAG_CNT][FLAGS][PAYLOAD...]`

| | 0 | 1 | 2 | 3 | 4-5 | 6 | 7 | 8 | 9+ |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | MAGIC | VER | MSG_ID | SEQ | LEN | FRAG_IDX | FRAG_CNT | FLAGS | PAYLOAD |
| **类型** | u8 | u8 | u8 | u8 | u16 BE | u8 | u8 | u8 | struct |
| **说明** | 0xAB | 0x01 | 消息类型 | 0-255 | PAYLOAD 字节数 | 固定 0 | CAN 分片数 | 填 0 | 见各章 |

- 线长 = **9 + LEN**（例：`agv_control_t` → 9+14=23 B）
- LEN 仅计 PAYLOAD，不含 9 字节头
- RS232 单帧：`FRAG_IDX=0`，`FRAG_CNT=1`
- CAN 多片：发送前设 `FRAG_CNT = ceil(线长 / 8)`，`FRAG_IDX` 仍为 0

### 0.2 RS232 模式（JETSON_LINK_CAN=0）

字节流 **混传**，首字节魔数分支（与现有 V3 并存）：

| 魔数 | 线长 | 用途 | 文档 |
|:---:|:---:|:---|:---|
| 0xAA | 24 B | 旧 V3 应用帧 | Jetson_RS232协议.md |
| 0xA5 | 11 B | 服务帧（时间同步/GPS/故障等） | 同上 §8 |
| 0xAB | 9+LEN | BLOB v2（本协议） | 本文 |

**0xAB 收包状态机**（与 V3 收 0xAA 类似）：

1. 读到 `0xAB` → 再收 8 字节（凑齐 9 字节头）
2. 从 byte 4-5 解析 LEN
3. 再收 LEN 字节 PAYLOAD
4. 按 MSG_ID 解 struct

| 项目 | 约定 |
|------|------|
| 物理 | USART2，PA2=TX / PA3=RX，115200 8N1 |
| 发送 | 整段 `[9B头][PAYLOAD]` 一次写出，**无 XOR** |
| 周期 | 见 §0.4 |

### 0.3 CAN 模式（JETSON_LINK_CAN=1）

与 V3 的 3×8 分片相同规则，只是线长可变：

1. 内存组装完整线格式 `wire[0 .. 9+LEN-1]`
2. 设 `wire[7] = FRAG_CNT = ceil((9+LEN) / 8)`
3. 选 CAN ID = `0x180 + MSG_ID`
4. 按顺序发 `ceil(线长/8)` 帧，每帧 8 B；末帧不足补 `0x00`
5. 接收端同 ID 顺序拼接后，再按 §0.1 解析

| MSG_ID | CAN ID | 方向 | struct | PAYLOAD | 线长 | CAN 帧数 |
|:---:|:---:|:---:|:---|:---:|:---:|:---:|
| 0x01 | 0x181 | Jetson→MCU | agv_control_t | 14 B | 23 B | 3 |
| 0x02 | 0x182 | MCU→Jetson | agv_motion_t | 40 B | 49 B | 7 |
| 0x03 | 0x183 | MCU→Jetson | mcu_status_t | 42 B | 51 B | 7 |
| 0x04 | 0x184 | MCU→Jetson | sensor_blob_t | 28 B | 37 B | 5 |
| 0x05 | 0x185 | MCU→Jetson | gps_compact_t | 32 B | 41 B | 6 |
| 0x06 | 0x186 | MCU→Jetson | agv_motor04_t | 44 B | 53 B | 7 |
| 0x07 | 0x187 | MCU→Jetson | agv_motor58_t | 44 B | 53 B | 7 |
| 0x08 | 0x188 | MCU→Jetson | agv_energy_t | 41 B | 50 B | 7 |
| 0x0B | 0x18B | MCU→Jetson | agv_motor_pos_t | 36 B | 45 B | 6 |
| 0x10 | 0x190 | Jetson→MCU | sensor_cfg_t | 8 B | 17 B | 3 |

`0x107~0x10B`（时间同步/故障/状态查询）仍走 `Jetson_CAN协议.md` 原定义，**不包在 0xAB 头里**。

### 0.4 建议通信周期

| 数据 | MSG | 周期 |
|------|:---:|:---:|
| Jetson 控制 | 0x01 | ≤20 ms |
| 运动摘要 | 0x02 | 20 ms |
| 电机 0-3 / 4-7 | 0x06 / 0x07 | 各 40 ms（交替） |
| 能源/里程 | 0x08 | 100 ms |
| GPS | 0x05 | 100 ms |
| 四路超声 | 0x04 | 20 ms |
| MCU 状态 | 0x03 | 50 ms |
| 全脉冲 | 0x0B | 1 Hz（可选） |
| 时间同步 | 0xA5/0x107 | 10 s（不变） |

**帧 stamp**：每个 struct byte 0-3 = `timestamp_ms`（u32 BE），必须有。

---

## 1. 底盘

### 1.1 控制 `agv_control_t`（MSG 0x01，14 B）

| | 0-3 | 4-5 | 6-7 | 8-9 | 10 | 11 | 12 | 13 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | linear_vel | angular_vel | steer_angle | control_mode | motion_drive_info | clear_fault | light_info |
| **类型** | u32 BE | s16 BE | s16 BE | s16 BE | u8 | u8 | u8 | u8 |
| **说明** | 帧时刻 | 线速度 mm/s | 角速度 0.001 rad/s | 转角 0.001 rad | 0 待机 1 CAN | 运动+驱动模式 | 清错码 | 灯光 |

### 1.2 运动 `agv_motion_t`（MSG 0x02，40 B）

| | 0-3 | 4 | 5 | 6 | 7-10 | 11-12 | 13-18 | 19-26 | 27-34 | 35-39 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | system_info | motion_info | light_pack | fault_code | bat_v | vel_x3 | wheel_angle_x4 | wheel_speed_x4 | rsv |
| **类型** | u32 BE | u8 | u8 | u8 | u32 BE | u16 BE | s16×3 BE | s16×4 BE | s16×4 BE | u8×5 |
| **说明** | 帧时刻 | 系统状态 | 运动模式 | 灯光+计数 | 0x211 故障 | 电压 0.1 V | 线角转 | 四轮角 | 四轮速 | 填 0 |

---

## 附录 A：C 结构体

见 MCU 侧 `APP/agv_blob/agv_blob_wire.h`；Jetson Python 实现见 `rs232_gateway/blob_codec.py`。

---

## 附录 B：变更记录

| 版本 | 日期 | 内容 |
|------|------|------|
| v2.0-draft.5.5 | 2026-06-15 | 新增 §0.2 RS232 / §0.3 CAN 双传输映射与 CAN ID 表 |
| v2.0-draft.5.4 | 2026-06-15 | 修复横排表分隔行列数不一致导致无法渲染 |
