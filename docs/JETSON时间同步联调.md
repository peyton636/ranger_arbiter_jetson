# Jetson ↔ STM32 时间同步联调

| 元数据 | 值 |
|--------|-----|
| **文档版本** | v1.3 |
| **日期** | 2026-06-12 |
| **状态** | RS232 **单调时钟对时** 实车通过；**UTC/业务打戳** 代码已接，户外 GPS 待验 |
| **联调结论** | gateway PING/QUERY/RTT ✅（2026-06-15）；`gps_rs232_to_fix` 已订阅 `offset_ms` + QUERY 锚点 |
| **工作区** | `~/catkin_ws/cangyirobot` |
| **关联协议** | [Jetson_CAN协议.md](./Jetson_CAN协议.md) §8.1（0x107/0x108） |
| **关联文档** | [JETSON_RS232_ROS2集成设计.md](./JETSON_RS232_ROS2集成设计.md) |
| **联调工具** | `tools/jetson_time_ping_test.py`（不依赖 ROS） |

本文档记录 Jetson 与 STM32B 之间 **时间同步链路** 的接线、协议、代码位置、测试步骤与常见问题。控制帧 V3 `0x01` 与时间同步 **独立并行**，互不干扰。

**要测什么、跑哪些命令**：见 **§7 测试实操手册**。

---

## 1. 两路串口分工（必读）

STM32B 上有 **两路 UART**，不要混用：

| MCU 口 | 接到哪 | 线上看到什么 | Jetson 是否使用 |
|--------|--------|--------------|-----------------|
| **USART1** PA9/PA10 | Windows 写程序电脑 | `[MOTION]`、`[BOOT]`、`[DS]` 等 **ASCII 日志** | **否** |
| **USART2** PA2/PA3 | Jetson RS232（USB-TTL） | V3 `0xAA` 二进制 + `0xA5` 服务帧 | **是** |

```text
Jetson USB-TTL TX  ──►  STM32 PA3 (USART2_RX)
Jetson USB-TTL RX  ◄──  STM32 PA2 (USART2_TX)
GND                ───  GND

Windows 调试线  ──►  PA9/PA10 (USART1)   ← 与时间同步无关，仅看人眼日志
```

Windows 上能看到 `[JETSON TIME] cmd=0x02/0x03`，说明 Jetson 下行到了 MCU；Jetson 上收不到 `0x108` 则查 **上行 PA2→Jetson RX** 或端口/占口。

---

## 2. Jetson USB 串口对照

插拔 USB 后 `ttyUSB` 编号 **可能变化**，优先用 **by-id**：

```bash
ls -la /dev/serial/by-id/
```

| 设备 | 典型端口 | 稳定路径 |
|------|----------|----------|
| **STM32B（Prolific）** | `ttyUSB7`（会变） | `/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0` |
| CH340（GPS/IMU 等） | `ttyUSB0` | 勿与时间同步混用 |
| Quectel 4G | `ttyUSB1`～`ttyUSB5` | 勿与时间同步混用 |

波特率：**115200 8N1**。

占口检查：

```bash
fuser -v /dev/ttyUSB7
pkill -f rs232_gateway
pkill -f jetson_bridge
```

---

## 3. 链路层：V3 + 服务帧

RS232 上 **混传** 两类帧（解析见 `rs232_gateway/service_frame.py`）：

| 魔数 | 长度 | 用途 |
|------|------|------|
| `0xAA` | 24 B | V3 控制/状态（`0x01` 下行，`0x02/0x03` 上行） |
| `0xA5` | 11 B | 服务帧：`[0xA5][ID_H][ID_L][8B payload]` |

时间同步使用 CAN ID（大端）：

| CAN ID | 方向 | 说明 |
|--------|------|------|
| **0x107** | Jetson → STM32 | 时间同步 **请求**（8B payload） |
| **0x108** | STM32 → Jetson | 时间同步 **响应**（8B payload） |

RS232 示例（QUERY）：

```text
TX: A5 01 07 01 00 00 00 00 00 00 00 00   # 0x107, CMD=0x01
RX: A5 01 08 xx xx xx xx yy yy yy yy ...  # 0x108, 8B 载荷
```

---

## 4. 时间同步命令（0x107 payload）

MCU 固件已实现 START / PING / STOP / QUERY，Jetson 侧实现于 `rs232_gateway/time_sync.py`。

| CMD | byte0 | payload 要点 | 0x108 回复格式 |
|-----|-------|--------------|----------------|
| **QUERY** | `0x01` | 其余填 0 | **格式 A**：byte0~3 `SYSTEM_TICK_MS`，byte4~7 `UTC_UNIX_SEC` |
| **START** | `0x02` | byte1 `session_id`；byte2~5 Jetson `mono_ms` u32 BE | **格式 B**：见下表 |
| **PING** | `0x03` | byte1 `seq`；byte2~5 `t1_ms` u32 BE；byte6~7 **必须 RSV=0** | **格式 B** |
| **STOP** | `0x04` | byte1 `session_id` | 一般不依赖回复 |

**格式 B**（START/PING 回复）：

| byte | 字段 |
|------|------|
| 0 | `cmd_echo`（`0x02` / `0x03`） |
| 1 | `seq_echo`（PING seq 或 START session） |
| 2~5 | `mcu_tick_rx` u32 BE |
| 6 | bit0 = `gps_utc_valid` |
| 7 | `proc_ms` × 0.1（MCU 处理耗时，可能为 0） |

**解析注意**：发 QUERY 时按 **格式 A** 解析；发 START/PING 时按 **格式 B** 解析。不要用 `data[0]==0x03` 去猜 QUERY 回复。

**时钟**：发 PING/START 时 `t1` 必须用 `time.monotonic()*1000`，不要用 `time.time()`。

---

## 5. RTT 与 offset

```python
t1 = time.monotonic() * 1000   # 发 PING/START 前
# ... 收到 0x108 ...
t4 = time.monotonic() * 1000   # 收到匹配 0x108 的时刻（不要等满超时再记 t4）

rtt_ms = t4 - t1
one_way = max(0, (rtt_ms - proc_ms) / 2)
offset_sample = mcu_tick_rx - t1 - one_way
offset_ms = 0.8 * offset_ms + 0.2 * offset_sample   # EMA
```

**换算**（给 ROS / 日志用）：

```text
jetson_mono_ms ≈ mcu_tick_ms - offset_ms
mcu_tick_ms    ≈ jetson_mono_ms + offset_ms
```

**offset 为大负数是否正常？** 正常。`mcu_tick` 为 MCU 开机后几千～几万 ms，`jetson_mono` 为系统运行总毫秒数（千万级），`offset = mcu_tick - jetson_mono` 会是很大的负数，不代表算错。

**真实 RTT**：RS232 上通常 **几 ms～十几 ms**。若脚本显示 RTT ≈ `--timeout`（如 1505ms），是 **等满超时才解析** 的测量假象；应用收到 0x108 的时刻算 t4（当前 `jetson_time_ping_test.py` 已修复）。

---

## 6. Jetson 代码结构

| 路径 | 作用 |
|------|------|
| `src/drivers/rs232_gateway/rs232_gateway/time_sync.py` | `JetsonTimeSync`：组帧、offset/RTT、START/PING/STOP/QUERY 流程 |
| `src/drivers/rs232_gateway/rs232_gateway/rs232_gateway_node.py` | 串口 50Hz V3 + 接入 TimeSync，发布 `/jetson_rs232/time_sync` |
| `src/drivers/rs232_gateway/rs232_gateway/service_frame.py` | `0xA5` / V3 混传解析、`encode_service_request()` |
| `src/drivers/rs232_gateway/rs232_gateway/service_codec.py` | 0x108 → `TimeSyncResponse` |
| `src/interfaces/jetson_can_msgs/msg/TimeSyncResponse.msg` | `system_tick_ms`、`utc_unix_sec`、`rtt_ms`、`offset_ms` 等 |
| `tools/jetson_time_ping_test.py` | **纯 Python 联调**（推荐先跑） |
| `tools/jetson_time_soak.py` | **1h soak** 监测 offset/RTT/V3 |
| `tools/jetson_rs232_link_test.py` | V3 链路 A/B（不含时间同步） |
| `src/drivers/gps_rs232_to_fix/gps_rs232_to_fix/time_stamp.py` | offset + QUERY UTC → `header.stamp` |

### 6.1 gateway 自动流程（`time_sync_enable:=true`）

1. 串口连接 → **START**(`session_id`, jetson_mono_ms)
2. 运行中 **1 Hz PING** 维持 offset、监测 RTT
3. 每 **10 s QUERY** 取 UTC（PING 回复里无 UTC）
4. 节点退出 → **STOP**

与 **50 Hz V3 下行** 同线程发送，互不阻塞。

### 6.2 编译

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_can_msgs rs232_gateway gps_rs232_to_fix --symlink-install
source install/setup.bash
```

### 6.3 ROS 运行与监视

```bash
ros2 launch rs232_gateway rs232_gateway.launch.py \
  serial_port:=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0 \
  time_sync_enable:=true

# 另一终端
ros2 topic echo /jetson_rs232/time_sync
```

| 话题 | 类型 | 说明 |
|------|------|------|
| `/jetson_rs232/time_sync` | `jetson_can_msgs/TimeSyncResponse` | tick、utc、`rtt_ms`、`offset_ms`、`cmd_echo`… |

| launch 参数 | 默认 | 说明 |
|-------------|------|------|
| `time_sync_enable` | true | 是否跑 START/PING/QUERY |
| `time_sync_ping_interval_s` | 1.0 | 运行中 PING 间隔 |
| `time_sync_query_interval_s` | 10.0 | QUERY 间隔 |
| `time_sync_session_id` | 1 | START/STOP session |
| `time_sync_rtt_warn_ms` | 50.0 | RTT 超过此值打 WARN |

---

## 7. 测试实操手册

按场景分步测试：**室内快验 → ROS 全链路 → 户外 UTC → 长稳 soak**。换线、换 USB 口或改代码后，至少跑 **7.1 + 7.2**。

### 7.0 每次测试前准备

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
source install/setup.bash

# 确认 STM32 串口（编号会变，优先 by-id）
ls -la /dev/serial/by-id/

# 确认无人占口
fuser -v /dev/ttyUSB7
# 若被占用：
pkill -f rs232_gateway
pkill -f jetson_bridge
```

硬件检查：

- STM32 已上电
- Jetson USB-TTL 接 **USART2 PA2/PA3**（不是 Windows 调试口 USART1）
- 波特率 **115200 8N1**

改代码后重新编译：

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_can_msgs rs232_gateway gps_rs232_to_fix --symlink-install
source install/setup.bash
```

常用端口变量（后续命令可复用）：

```bash
export PORT=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
# 或：export PORT=/dev/ttyUSB7
```

---

### 7.1 场景 A：室内快验（纯 Python，约 5 分钟）

**目的**：确认接线、PING/QUERY、offset/RTT 正常；**不依赖 ROS、不依赖 GPS**。

```bash
cd ~/catkin_ws/cangyirobot

# 1. 侦听：应有大量 V3 0xAA，不是 ASCII 日志
python3 tools/jetson_time_ping_test.py --port "$PORT" --probe

# 2. QUERY（验证固件 + 上行）
python3 tools/jetson_time_ping_test.py --port "$PORT" --query-only

# 3. 仅 PING（排除 START 副作用）
python3 tools/jetson_time_ping_test.py --port "$PORT" --ping-only --count 10 --timeout 1.5

# 4. 全流程 START + PING + QUERY
python3 tools/jetson_time_ping_test.py --port "$PORT" --count 20 --query --timeout 1.5
```

| 步骤 | 看什么 | 通过标准 |
|------|--------|----------|
| `--probe` | 上行帧类型 | 大量 `V3 0xAA` 帧；ASCII 占比低（&lt;30%） |
| `--query-only` | QUERY 回复 | `QUERY ok: mcu_tick=... utc_unix=0`（室内 utc=0 正常） |
| `--ping-only` | PING 回复 | 10 次 `echo=0x03`，seq 递增，`fail=0`，RTT 约几十 ms |
| 全流程 | START/PING/QUERY | `START OK cmd_echo=0x02`；PING 全通；`QUERY OK` |
| Windows USART1（可选） | 人眼日志 | 出现 `[JETSON TIME] cmd=0x02` / `cmd=0x03` |

---

### 7.2 场景 B：ROS 全链路（室内，验 gateway + `/fix`）

**目的**：验证 `rs232_gateway` 发布 `time_sync`，`gps_rs232_to_fix` 能跑并订阅 offset。

**终端 1 — 启动 gateway**

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash

ros2 launch rs232_gateway rs232_gateway.launch.py \
  serial_port:=/dev/ttyUSB7 \
  time_sync_enable:=true
# 或用稳定路径：
# serial_port:=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
```

**终端 2 — 监视 time_sync**

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash

ros2 topic echo /jetson_rs232/time_sync
```

**终端 3 — GPS → `/fix`**

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash

ros2 launch gps_rs232_to_fix gps_rs232_to_fix.launch.py
ros2 topic echo /fix
```

| 看什么 | 通过标准 |
|--------|----------|
| `time_sync` PING | 约 1 Hz，`cmd_echo=3`，`rtt_ms` ~40ms，`offset_ms` 大负数但稳定 |
| `time_sync` QUERY | 每 10 s 一次；室内 `utc_unix_sec=0` |
| `/fix` | 有 GPS 帧则发布；室内多为 `status=NO_FIX` |
| `gps_rs232_to_fix` 日志 | 启动行含 `stamp=time_sync+UTC锚点` |
| `header.stamp.sec`（室内） | 仍为 ROS 时钟（约 17 亿级），**不要当 UTC** |

---

### 7.3 场景 C：户外 UTC 打戳 + MCU QUERY（需 GPS fix）

**目的**：`/fix.header.stamp` 与墙钟 UTC 误差 &lt;100 ms（粗验）；MCU QUERY 与 Jetson 一致。

先按 **7.2** 起 gateway + `gps_rs232_to_fix`，到 **户外有 GPS fix** 处：

```bash
# QUERY 应有非 0 UTC
ros2 topic echo /jetson_rs232/time_sync --field utc_unix_sec

# 对比墙钟（秒级粗比）
ros2 topic echo /fix --field header.stamp
date +%s

# 纯 Python 快速看 QUERY（需先停 gateway，避免占口）
python3 tools/jetson_time_ping_test.py --port "$PORT" --query-only
```

| 看什么 | 通过标准 |
|--------|----------|
| QUERY `utc_unix_sec` | ≠ 0，每 10 s 递增合理 |
| `/fix` | `status=FIX`，lat/lon 有效 |
| `header.stamp.sec` | 接近 `date +%s`（百毫秒级误差可接受） |
| MCU LCD / Windows | `GPS_UTC=1`（`GPS_PrintStatus`），与 Jetson QUERY UTC 一致 |

打戳公式（`gps_rs232_to_fix` 已实现）：

```text
t_mcu_ms  ≈ jetson_mono_ms + offset_ms
utc_sec   ≈ utc_unix_sec + (t_mcu_ms - query_tick_ms) / 1000
```

Launch 参数：`use_time_sync_stamp:=true`（默认开启）。

---

### 7.4 场景 D：1 h soak（验长稳）

**目的**：offset 无漂移趋势、RTT WARN 少、V3 上行持续。

**前提**：**gateway 必须在另一终端持续运行**，soak 只订阅话题、不占串口。若先 `Ctrl+C` 停了 gateway 再跑 soak，会出现 `ping=0`（无效结果）。

```text
终端1（保持运行，不要关）: ros2 launch rs232_gateway ...
终端2: python3 tools/jetson_time_soak.py --duration 300
```

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
source install/setup.bash

# 5 分钟试跑
python3 tools/jetson_time_soak.py --duration 300

# 正式 1 小时
python3 tools/jetson_time_soak.py --duration 3600
```

结束时看脚本汇总：

| 指标 | 期望 | 验收 |
|------|------|------|
| `offset_ms` 漂移 | ±几 ms，无单调趋势 | span &lt; 50 ms |
| RTT WARN（&gt;50 ms） | 偶发 | &lt; 5% PING |
| V3 上行 | PING 后仍持续 | &gt; 15 Hz（probe 约 ~25 Hz） |

---

### 7.5 建议测试顺序总表

| 顺序 | 场景 | 章节 | 耗时 | 何时必做 |
|------|------|------|------|----------|
| 1 | 室内快验（纯 Python） | §7.1 | ~5 min | 换线/换口/改串口相关代码后 |
| 2 | ROS gateway + time_sync | §7.2 | ~5 min | 改 gateway/time_sync 后 |
| 3 | ROS + gps_rs232_to_fix 室内 | §7.2 | ~5 min | 改 GPS 打戳后 |
| 4 | 户外 UTC + MCU QUERY | §7.3 | 半日 | 验 UTC 打戳、QUERY |
| 5 | 1 h soak | §7.4 | 1 h | 上线前长稳 |

---

### 7.6 V3 基础链路（与时间同步分开测）

```bash
python3 tools/jetson_rs232_link_test.py --port "$PORT" --listen-only --time 5
python3 tools/jetson_rs232_link_test.py --port "$PORT" --time 5
```

---

## 8. 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `could not open port` / `$PORT` 为空 | 未设置环境变量 | 用 `--port /dev/ttyUSB7` 或先 `export PORT=...` |
| `--probe` 0 字节 | 未上电、接线错、端口错 | 查 PA2/PA3、Prolific by-id |
| probe 大量 ASCII | 接到 USART1 调试口 | 改接 USART2 |
| QUERY 超时、probe 正常 | 未发 QUERY 或固件未回 0x108 | Windows 是否有 `[JETSON TIME]` |
| START 超时、QUERY 成功 | 未烧 START/PING 新固件 | 烧录后 Windows 应见 `cmd=0x02` |
| Windows 有 `cmd=0x03`、Jetson PING 超时 | 超时太短、V3 混传难找 0x108 | `--timeout 1.5`；用新版测试脚本 |
| START 后 PING 收 0 字节 | 旧脚本 0.5s + 未清缓冲 | 加长 timeout；发前 `reset_input_buffer` |
| `offset` 千万级负数 | 正常尺度差 | 用 `mcu_tick` 差值验证映射 |
| RTT ≈ 1500ms | 测量方式问题 | 收到 0x108 立即记 t4；真实 RTT 应 &lt;30ms |
| `utc_unix=0` | 室内无 GPS fix | 正常；有 fix 后 QUERY 应非 0 |
| 室内 `header.stamp` 仍 17 亿级 | 无 UTC 锚点 | 正常；户外有 fix 后应接近墙钟 |
| soak 无数据 | gateway 未启动 | 先 `ros2 launch rs232_gateway` |

### Windows 与 Jetson 对照

| Windows USART1 | Jetson | 结论 |
|----------------|--------|------|
| 无 `[JETSON TIME]` | 任意 | Jetson 下行未到 USART2 |
| 有 `cmd=0x02`，Jetson START 超时 | 上行问题或解析 | 查 PA2→RX、脚本 timeout |
| 有 `cmd=0x03`，Jetson PING 超时 | 同上或超时太短 | 加长 timeout |
| QUERY 通、START/PING 不通 | 固件 QUERY 路径 OK，快路径未通 | 查 MCU 中断里 PING 回包 |

---

## 9. 与整车业务的关系

| 模块 | 说明 |
|------|------|
| V3 `0x01` 控制 | `agv_base_driver` → `/jetson_rs232/command`，50Hz，与时间同步 **独立** |
| V3 `0x02/0x03` 状态 | gateway 照常解析发布 |
| GPS `0x104~0x106` | `gps_rs232_to_fix` 订阅 `/jetson_rs232/time_sync`，用 offset + QUERY UTC 锚点打 `/fix.header.stamp` |
| 故障 `0x109` | 订阅即可 |

**注意**：`/fix.header.stamp` 或 `time_sync.header.stamp.sec` 若为 **17 亿级** 是 ROS/仿真时钟，**不是 UTC**；户外有 fix 时应为真实 Unix 秒（约 1.7×10⁹ 且与墙钟一致）。

**仍待 Phase2**：V3 byte14~17 硬件打戳；IMU 融合节点消费 offset。

---

## 10. 下一步工作（2026-06-15 起）

### 10.1 Jetson gateway（实车 2026-06-15）

| 项 | 状态 | 说明 |
|----|------|------|
| 接入 `rs232_gateway`，发布 `/jetson_rs232/time_sync` | ✅ 实车验证通过 | `rs232_gateway_node.py` + `TimeSyncResponse` |
| TimeSync 收到 **0x108 瞬间**记 `t4` | ✅ 实车验证通过 | 解析到 0x108 时立即 `mono_ms()` 传入 `on_response` |
| 运行期 **1 Hz PING** + **10 s QUERY** | ✅ 实车验证通过 | `JetsonTimeSync.tick()`，连接时 START + burst |
| RTT 告警（**> 50 ms**） | ✅ 实车验证通过 | 实车 RTT ~40ms，偶发 ~60ms WARN |
| GPS 消费 `offset_ms` + UTC 锚点打戳 | ✅ 已实现 | `gps_rs232_to_fix` + `time_stamp.py`；户外 UTC 误差待验 |
| 日志时间轴统一 | ⬜ 待做 | 见 §10.3 |

实车观测（室内无 GPS）：`offset_ms` ≈ -17844302（大负数正常）；`cmd_echo=3` PING ~1Hz；QUERY `utc_unix_sec=0`。

验证 gateway：

```bash
ros2 launch rs232_gateway rs232_gateway.launch.py serial_port:=/dev/ttyUSB7
ros2 topic echo /jetson_rs232/time_sync
# 应约 1Hz 见 PING（cmd_echo=0x03），10s 见 QUERY
```

### 10.4 GPS 打戳 + 1 h soak

完整命令见 **§7.3（户外 UTC）**、**§7.4（soak）**。

#### 10.4.1 `/fix` UTC 打戳（`gps_rs232_to_fix`）

订阅 `/jetson_rs232/time_sync`，在发布 NavSatFix 时：

```text
t_mcu_ms   ≈ jetson_mono_ms + offset_ms     # offset = mcu_tick - jetson_mono
utc_sec    ≈ utc_unix_sec + (t_mcu_ms - query_tick_ms) / 1000   # 有 QUERY UTC 锚点时
```

无 UTC 锚点（室内 `utc_unix_sec=0`）时回退 ROS 时钟，**不声称 UTC 精度**。

Launch 参数：`use_time_sync_stamp:=true`（默认开启）。

#### 10.4.2 1 h soak（`tools/jetson_time_soak.py`）

见 §7.4。

#### 10.4.3 MCU 户外 GPS QUERY（半日）

见 §7.3 验收表；纯 Python：`python3 tools/jetson_time_ping_test.py --port "$PORT" --query-only`

### 10.2 MCU 侧（可选）

| 项 | 状态 | 说明 |
|----|------|------|
| GPS → RTC | ⬜ | 固件侧 |
| Phase2 V3 打戳（byte 布局待定） | ⬜ | 0x102 字段与 sonar 占用需协调 |
| CAN 模式复用 `HandleTimeRequest` | ⬜ | RS232 已通，CAN 路径对称实现 |
| `proc_ms` byte7 填充 | ⬜ 可选 | 当前常为 0 |

### 10.3 整车

| 项 | 状态 | 说明 |
|----|------|------|
| 日志时间轴统一（MCU tick ↔ Jetson mono ↔ UTC） | ⬜ | GPS 打戳已接，整车日志对齐待做 |
| 1 h soak 测漂移 | ✅ 工具就绪 | `tools/jetson_time_soak.py`；实车 1h 记录待跑 |

---

## 11. 命令速查

完整步骤见 **§7 测试实操手册**。常用命令：

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
source install/setup.bash

export PORT=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0

# --- 7.1 室内快验 ---
python3 tools/jetson_time_ping_test.py --port "$PORT" --probe
python3 tools/jetson_time_ping_test.py --port "$PORT" --query-only
python3 tools/jetson_time_ping_test.py --port "$PORT" --ping-only --count 10 --timeout 1.5
python3 tools/jetson_time_ping_test.py --port "$PORT" --count 20 --query --timeout 1.5

# --- 7.2 ROS 全链路 ---
ros2 launch rs232_gateway rs232_gateway.launch.py serial_port:=/dev/ttyUSB7
ros2 topic echo /jetson_rs232/time_sync
ros2 launch gps_rs232_to_fix gps_rs232_to_fix.launch.py
ros2 topic echo /fix

# --- 7.3 户外 UTC 粗验 ---
ros2 topic echo /jetson_rs232/time_sync --field utc_unix_sec
ros2 topic echo /fix --field header.stamp
date +%s

# --- 7.4 soak（gateway 需先启动）---
python3 tools/jetson_time_soak.py --duration 300
python3 tools/jetson_time_soak.py --duration 3600
```

---

## 12. 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-06-15 | 初稿：START/PING/STOP/QUERY、`time_sync.py`、联调步骤与故障表 |
| v1.1 | 2026-06-15 | §10 下一步工作；gateway t4 瞬时记录；RTT 告警默认 50ms |
| v1.2 | 2026-06-12 | §10.1 实车通过；`gps_rs232_to_fix` offset/UTC 打戳；§10.4 soak 工具 |
| v1.3 | 2026-06-12 | 新增 §7 测试实操手册（准备/A/B/C/D 场景命令与验收表） |
