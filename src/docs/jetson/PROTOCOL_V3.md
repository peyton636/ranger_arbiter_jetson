# Jetson ↔ STM32B ↔ STM32A 通信协议规范

**版本：V3.0（定稿目标）**  
**状态：协议先行 — 实现须与本文件一致；当前代码为 Legacy，见附录 A。**

| 角色 | 节点 | 说明 |
|------|------|------|
| 大脑 | Jetson (ROS2) | 规划/遥控 → 24B 下行；解析 24B 上行 |
| 仲裁 | STM32B | 超声限速、心跳检测、Jetson↔CAN 映射 |
| 底盘 | STM32A / Ranger Mini | 官方 CAN 协议，500K MOTOROLA |

---

## 0. 版本与迁移策略

| 版本 | Jetson↔B | 说明 |
|------|----------|------|
| **V3.0** | 24B，`0xAA` 头 | **本文件唯一实现目标** |
| Legacy V1 | 8B `0xFF` 指令 + 12B `0xFF` 传感器 | 仅联调过渡，**禁止与新 V3 混用同一固件解析路径** |

**迁移完成条件（全部满足后删除 Legacy 代码路径）：**

1. STM32B 稳定解析 V3 下行（`frame_type=0x01`），`seq` 超时 300ms 行为符合第 6 节。  
2. STM32B 周期发送 V3 上行 `0x02`（20ms），超声数据来自 B 内部采集（不再依赖独立 12B 帧作为主路径）。  
3. Jetson `ds_jetson_bridge` 仅发 V3 24B；`ds_serial_monitor` 12B 改为可选调试工具。  
4. B↔A CAN 上 **0x111 仅用于运动指令**（与 Ranger 手册一致），不在此 ID 上发超声距离。

---

## 1. 物理层

| 链路 | 接口 | 波特率 | 格式 |
|------|------|--------|------|
| Jetson ↔ STM32B | UART (RS232/USB 串口) | **115200** | 8N1，**大端** |
| STM32B ↔ STM32A | CAN 2.0B | **500 kbit/s** | **MOTOROLA**（多字节信号高字节在前） |

**Jetson 串口设备：** 以 `Prolific` USB 转串口为准（现场多为 `/dev/ttyUSB5`）；**不得**使用 Quectel 模组口发 STM32 指令。

---

## 2. Jetson ↔ STM32B（V3.0 统一帧）

### 2.1 帧外壳（固定 24 字节）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | `header` | u8 | 固定 **`0xAA`** |
| 1 | `frame_type` | u8 | `0x01` 下行，`0x02` 上行状态，`0x03` 扩展上行 |
| 2 | `seq` | u8 | 0~255 循环；**每帧递增**（含心跳） |
| 3~22 | `payload` | 20B | 见下文 |
| 23 | `xor` | u8 | `payload[0..22]` 即 **Byte0~Byte22** 逐字节异或 |

**校验：** `xor = B0 ^ B1 ^ … ^ B22`。

**收发周期：**

| 方向 | 建议周期 | 超时 |
|------|----------|------|
| Jetson → B 下行 | **20~50 ms**（推荐 20ms / 50Hz） | B 侧 **300ms** 无有效下行 → `link_state.bit0=1` |
| B → Jetson `0x02` | **20 ms** | Jetson 可选 300ms 监看 `seq` |
| B → Jetson `0x03` | **20 ms** 或与 `0x02` 交替 | 可选 |

---

### 2.2 下行 `frame_type = 0x01`（Jetson → B）

| 偏移 | 字段 | 类型 | 范围 / 取值 | 说明 |
|------|------|------|-------------|------|
| 3 | `mode_req` | u8 | 0 / 1 / 2 | 0=待机，1=**CAN 控制**，2=遥控（B 不切换遥控，仅状态） |
| 4~5 | `v_cmd` | s16 BE | ±2000 | 线速度 **mm/s** |
| 6~7 | `omega_cmd` | s16 BE | ±3259 | 角速度 **0.001 rad/s** |
| 8~9 | `steer_cmd` | s16 BE | ±698（阿克曼） | 转向角 **0.001 rad**；斜移模式 ±1571 |
| 10 | `motion_model` | u8 | 0~3 | 0=阿克曼，1=斜移，2=自旋，3=驻车 |
| 11 | `light_en` | u8 | 0 / 1 | 灯光使能 |
| 12 | `light_mode` | u8 | 0 / 1 | 0=常关，1=常开 |
| 13 | `clear_error` | u8 | 见 §2.5 | 清错脉冲（按需置位，平时 0） |
| 14~22 | `reserved` | — | **0** | 预留 |
| 23 | `xor` | u8 | — | 帧尾 |

**Jetson 缺省安全行为：**

- 无 `/cmd_vel` 或超时 **200ms**：`v_cmd=0`，`omega_cmd=0`，`seq` 仍递增。  
- 恢复：**连续 1s** 仅发零速后，才接受新运动指令（与 B 仲裁恢复对齐）。  
- 上电先发 `mode_req=1` 若干帧，再发运动（B 负责映射 **0x421**）。

**与 ROS `/cmd_vel` 换算：**

```text
v_cmd_mm_s     = clamp(round(linear.x * 1000), ±2000)
omega_cmd      = clamp(round(angular.z * 1000), ±3259)   // 单位 0.001 rad/s
steer_cmd      = 默认 0（阿克曼随动由底盘闭环，或由上层显式填）
```

---

### 2.3 上行 `frame_type = 0x02`（B → Jetson，运动 + 超声 + 电池）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 3 | `safety_state` | u8 | 1=正常，2=限速，3=降级，4=紧急 |
| 4 | `link_state` | u8 | bit0：Jetson 链路（0=正常，1=丢失）；bit1：CAN 链路（0=正常，1=丢失） |
| 5 | `limit_factor` | u8 | 0~100，当前限速百分比 |
| 6~7 | `v_actual` | s16 BE | 实际线速度 **mm/s**（由 0x221 换算，见 §4.2） |
| 8~9 | `omega_actual` | s16 BE | **0.001 rad/s** |
| 10~11 | `steer_actual` | s16 BE | **0.001 rad** |
| 12~13 | `sonar_front` | u16 BE | 前超声 **mm**，无效 = `0xFFFF` |
| 14~15 | `sonar_back` | u16 BE | 后 |
| 16~17 | `sonar_left` | u16 BE | 左 |
| 18~19 | `sonar_right` | u16 BE | 右 |
| 20~21 | `battery_voltage` | u16 BE | **0.1 V** |
| 22 | `battery_soc` | u8 | 0~100 % |
| 23 | `xor` | u8 | — |

**超声通道映射（B 内部）：** IF1=前，IF2=后，IF3=左，IF4=右。

---

### 2.4 上行 `frame_type = 0x03`（B → Jetson，四轮明细）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 3~4 | `wheel_speed_rf` | s16 BE | 右前 mm/s |
| 5~6 | `wheel_speed_rr` | s16 BE | 右后 |
| 7~8 | `wheel_speed_lr` | s16 BE | 左后 |
| 9~10 | `wheel_speed_lf` | s16 BE | 左前 |
| 11~12 | `steer_angle_rf` | s16 BE | 0.001 rad |
| 13~14 | `steer_angle_rr` | s16 BE | |
| 15~16 | `steer_angle_lr` | s16 BE | |
| 17~18 | `steer_angle_lf` | s16 BE | |
| 19 | `motor_temp_max` | s8 | 最高电机温度 ℃（来自 0x261 聚合） |
| 20 | `driver_state` | u8 | 驱动器状态聚合（见 Ranger 表 2） |
| 21~22 | `reserved` | — | 0 |
| 23 | `xor` | u8 | — |

---

### 2.5 `clear_error`（下行 byte13，与 CAN 0x441 一致）

| 值 | 含义 |
|----|------|
| 0x00 | 清除全部非严重故障（含急停恢复后清除） |
| 0x01~0x08 | 清除 1~8 号电机驱动器通讯故障 |
| 0x09 | 清除电池欠压 |
| 0x0a | 清除遥控信号丢失 |
| 0x0b~0x0e | 清除 5~8 号转向校准故障 |
| 0x0f | 清除过流 |
| 0x10 | 清除过温 |

**用法：** 仅在需要时单帧置位；常态保持 **0**。

---

## 3. STM32B ↔ STM32A（Ranger Mini CAN）

> 与官方《CAN 接口协议》一致；B 为 CAN 主站发送控制帧，A（底盘）周期反馈。

### 3.1 CAN ID 表（实现必备）

| CAN ID | 方向 | 周期 | DLC | 说明 |
|--------|------|------|-----|------|
| 0x111 | B→A | 20ms | 8 | 运动控制 |
| 0x121 | B→A | 20ms | 8 | 灯光 |
| 0x141 | B→A | 按需 | 1 | 运动模型 |
| 0x421 | B→A | 按需 | 1 | 控制模式 |
| 0x423 | B→A | 按需 | 1 | 电流/电压驱动模式 |
| 0x441 | B→A | 按需 | 1 | 错误清除 |
| 0x211 | A→B | 20ms | 8 | 系统状态 |
| 0x221 | A→B | 20ms | 8 | 运动回馈 |
| 0x231 | A→B | 20ms | 8 | 灯光反馈 |
| 0x241 | A→B | 20ms | 8 | 遥控器信息 |
| 0x251~0x258 | A→B | 20ms | 8 | 电机高速（1~8 号） |
| 0x261~0x268 | A→B | 100ms | 8 | 电机低速 |
| 0x271 | A→B | 20ms | 8 | 四轮转角 |
| 0x281 | A→B | 20ms | 8 | 四轮转速 |
| 0x291 | A→B | 20ms | 3 | 运动模式回馈 |
| 0x311 | A→B | 20ms | 8 | 前轮里程 |
| 0x312 | A→B | 20ms | 8 | 后轮里程 |
| 0x361 | A→B | 500ms | 8 | BMS 数据 |
| 0x362 | A→B | 500ms | 4 | BMS 告警 |

**禁止：** 在 B↔A 总线上用 **0x111** 发送超声距离（与运动指令冲突）。

---

### 3.2 运动控制 `0x111`（B→A）

| Byte | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0~1 | `v_cmd` | s16 | mm/s，±2000（大角时 ±700，按手册） |
| 2~3 | `omega_cmd` | s16 | 0.001 rad/s，±3259 |
| 4~5 | reserved | — | 0 |
| 6~7 | `steer_cmd` | s16 | 0.001 rad |

**接收超时（底盘侧）：** 500ms 无 0x111 → 底盘判失联。

---

### 3.3 系统状态 `0x211`（A→B）

| Byte | 字段 | 说明 |
|------|------|------|
| 0 | `chassis_state` | 0x00 正常，0x02 异常 |
| 1 | `mode_fb` | 0x00 待机，0x01 CAN，0x03 遥控 |
| 2~3 | `battery_voltage` | u16，**0.1V** |
| 4~7 | `fault_code` | u32 故障位（见官方故障表） |

---

### 3.4 运动回馈 `0x221`（A→B）

| Byte | 字段 | 说明 |
|------|------|------|
| 0~1 | `v_actual` | s16，**实际速度 ×1000，单位 0.001 m/s** |
| 2~3 | `omega_actual` | s16，0.001 rad/s |
| 4~5 | reserved | 0 |
| 6~7 | `steer_actual` | s16，0.001 rad |

**换算到 Jetson 上行 `v_actual`（mm/s）：**

```text
v_mm_s = raw_0x221   // 因 0.001 m/s × 1000 = mm/s，数值上与 mm/s 同量纲
```

例：0.15 m/s 前进 → raw = 150 → Jetson 填 `v_actual = 150` mm/s。

---

### 3.5 控制模式 `0x421`（B→A）

| Byte | 值 | 含义 |
|------|-----|------|
| 0 | 0x00 | 待机（上电默认） |
| 0 | 0x01 | **CAN 指令模式**（Jetson 控车前置条件） |

**注意：** 遥控器连接时具最高权限，可屏蔽 CAN；`mode_fb==0x03` 时 Jetson 应停止发非零运动。

---

### 3.6 运动模型 `0x141` / 回馈 `0x291`

| 值 | 模式 |
|----|------|
| 0x00 | 前后阿克曼（上电默认） |
| 0x01 | 斜移 |
| 0x02 | 自旋 |
| 0x03 | 驻车 |

切换过程中（`0x291.byte1==0x01`）**不响应**速度指令。

---

### 3.7 电机编号（与手册一致）

| 编号 | 位置 |
|------|------|
| 1 | 右前轮 |
| 2 | 右后轮 |
| 3 | 左后轮 |
| 4 | 左前轮 |
| 5 | 右前转向 |
| 6 | 右后转向 |
| 7 | 左后转向 |
| 8 | 左前转向 |

---

## 4. STM32B 内部映射

### 4.1 下行（Jetson → B → A）

| Jetson V3 字段 | CAN | 字段 |
|----------------|-----|------|
| `mode_req` | 0x421 | byte0 |
| `v_cmd`, `omega_cmd`, `steer_cmd` | 0x111 | byte0~7 |
| `motion_model` | 0x141 | byte0 |
| `light_en`, `light_mode` | 0x121 | byte0~1 |
| `clear_error` | 0x441 | byte0 |

**仲裁后** 方可发 0x111（限速/降级/紧急时由 B 改写 `v_cmd`/`omega_cmd`）。

---

### 4.2 上行（A → B → Jetson）

| CAN | 字段 | Jetson V3 字段 |
|-----|------|----------------|
| 0x221 | v, ω, steer | `0x02`: v_actual, omega_actual, steer_actual |
| 0x211 | battery_voltage | `0x02`: battery_voltage |
| 0x361 | soc | `0x02`: battery_soc |
| 0x281 | wheel_speed_* | `0x03`: wheel_speed_* |
| 0x271 | steer_* | `0x03`: steer_angle_* |
| 0x261 | temp, driver_state | `0x03`: motor_temp_max, driver_state |
| 0x211 | fault_code | `link_state.bit1`（规则：见 §6.2） |
| B 超声采集 | — | `0x02`: sonar_* |

---

## 5. STM32B 仲裁状态机（定稿）

**距离单位：mm。** 阈值：

| 符号 | 值 | 含义 |
|------|-----|------|
| `D_EMERG` | **300** | <30cm，紧急 |
| `D_LIMIT` | **800** | <80cm 且 ≥30cm，限速 |
| `D_CLEAR` | **800** | >80cm，正常 |

**限速系数（`limit_factor` 0~100）：**

```text
factor = clamp((dist_min_mm - 300) / 500.0, 0.0, 1.0)
v_cmd_out = round(v_cmd_in * factor)
```

其中 `dist_min_mm` 为四向有效超声最小值（无效通道不参与）。

### 5.1 真值表

| Jetson `seq` 有效 | dist_min | safety_state | link bit0 | B 行为 |
|-------------------|----------|--------------|-----------|--------|
| 是 | >800 | 1 | 0 | 透传指令 |
| 是 | 300~800 | 2 | 0 | 比例限速 |
| 是 | <300 | 4 | 0 | 强制 v=0, ω=0 |
| 否 | >800 | 3 | 1 | 降级巡游 v=300 mm/s |
| 否 | 300~800 | 3 | 1 | 降级避障（后退+转向，策略由 B 实现） |
| 否 | <300 | 4 | 1 | 强制零速 |

**固件实现说明：** 当前 STM32B 在 `DEGRADED`/`SPEED_LIMIT` 下使用 `Arbiter_ProcessDirectionalPolicy()` 四向 if-else 策略，**未**使用上表「丢失心跳 + 远距 v=300mm/s」简化巡游。行为以固件为准，详见 `docs/STM32B_FIRMWARE_NOTES.md`。

**Jetson 链路丢失判定：** 300ms 内未收到合法下行帧（`header==0xAA`, `type==0x01`, xor 正确）或 `seq` 未更新。

**CAN 链路丢失判定（bit1）：** 500ms 未收到 0x221（或 0x211 `chassis_state` 异常 — 实现二选一，B 固件注释写明）。

---

## 6. 超时与周期汇总

| 参数 | 值 |
|------|-----|
| Jetson 下行周期 | 20~50 ms |
| Jetson 下行超时（B 判心跳） | **300 ms** |
| Jetson `/cmd_vel` 本地超时 | **200 ms** → 发零速 |
| Jetson 恢复清零保持 | **1000 ms** |
| CAN 0x111 底盘超时 | **500 ms** |
| B 上行 `0x02` / `0x03` | **20 ms** |
| 超声采集（B 内部） | **20 ms** |

---

## 附录 A — Legacy（过渡，勿与 V3 混用）

### A.1 Jetson → B：8 字节（V1，已废弃）

| Byte | 内容 |
|------|------|
| 0 | 0xFF |
| 1~2 | v mm/s BE |
| 3~4 | ω **0.01 rad/s** BE |
| 5 | 0xAA 心跳 |
| 6 | 0 |
| 7 | XOR(0..6) |

### A.2 B → Jetson 传感器：12 字节（USART3，已废弃）

| Byte | 内容 |
|------|------|
| 0 | 0xFF |
| 1 | sensor_count (=4) |
| 2~9 | 4×uint16 BE 距离 mm |
| 10 | beep % |
| 11 | XOR |

### A.3 CAN 调试帧（非 Ranger，逐步淘汰）

| ID | 用途 |
|----|------|
| 0x110 | B 调试状态 |
| 0x111 | **曾用于** 四路超声 CAN 上报 — **与 Ranger 0x111 运动指令冲突，V3 禁止在 B↔A 总线保留** |

---

## 附录 B — 实现检查清单（代码阶段）

**STM32B**

- [ ] UART 收 24B 状态机 + `seq`/300ms  
- [ ] 映射发 0x421/0x111/0x141/0x121/0x441  
- [ ] 收 0x221/0x211/0x281/0x271/0x361，填 0x02/0x03  
- [ ] 仲裁表 §5  
- [ ] 删除或 `#if LEGACY` 8B/12B 主路径  

**Jetson**（`ds_jetson_bridge` v0.2，见 `docs/STM32B_FIRMWARE_NOTES.md`）

- [x] `jetson_protocol` → 24B encode/decode  
- [x] `jetson_bridge`：50Hz 下行，`seq++`，订阅 `/cmd_vel`  
- [x] 解析 0x02 → `stm32b/*` 话题  
- [ ] 解析 0x03 扩展帧（可选）  
- [x] launch：`cmd_port` 默认 ttyUSB5  
- [ ] `ds_can_monitor` 注释：仅监听 A↔B 或调试口，勿解析 0x111 为超声  

**联调顺序**

1. B 单测：V3 下行 → CAN 0x421+0x111，底盘使能  
2. Jetson 发零速 + `seq` 变化，B `link_state.bit0=0`  
3. teleop → 运动；超声手挡验证 safety_state 2/4  
4. 断 Jetson 300ms 验证降级 state 3  

---

## 附录 C — 待定项（实现前需硬件确认一条）

| 项 | 建议默认 | 确认方 |
|----|----------|--------|
| Jetson USB 线接 B 的 USART2 还是 USART3 | 统一 USART2 全双工 24B | 硬件 |
| 单根 USB 线 | `cmd_port` = `sensor_port` = ttyUSB5 | 现场 |
| 降级巡游 300mm/s 方向 | 沿上次有效运动方向 | 算法 |

**确认后** 将附录 C 表格改为固定值，版本升为 **V3.0.1**。

---

*文档路径：`~/catkin_ws/docs/PROTOCOL_V3.md` — 后续代码 PR 须引用本节号（§2.2、§5.1 等）。*
