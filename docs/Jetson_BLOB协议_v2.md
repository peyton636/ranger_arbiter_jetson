# Jetson 与 MCU BLOB 二进制协议（v2.0 草案）

| 元数据 | 值 |
|--------|-----|
| **协议版本** | v2.0-draft.5.6 |
| **文档日期** | 2026-06-16 |
| **物理层** | RS232 115200 8N1（USART2）或 CAN2 500 kbps（编译期二选一） |
| **编码** | 多字节整数 **大端 BE**；`#pragma pack(1)` |

> 横排表约定：与 `Jetson_RS232协议.md` 相同——首列留空；多字节范围写 `4-5`（ASCII 连字符）；分隔行 `|:---:|` 个数必须与列数一致。

**与 V3 的关系**：BLOB PAYLOAD（struct）为 v2 新定义；传输层沿用 V3 思路——同一条线格式，RS232 整帧发送，CAN 按 ID 8 字节顺序分片。时间同步等 `0xA5` / `0x107~0x108` 服务帧保持不变，与 BLOB 混传。

硬件切换见 `硬件连接与通信协议.md` §2.2（`JETSON_LINK_CAN` 1=CAN2，0=USART2）。

**Jetson ROS2 实现**：`rs232_gateway` 包，`use_blob_v2:=true`（默认）时发 `0xAB` MSG `0x01`，解析上行 BLOB 并映射到 `/jetson_rs232/v3_status` 等 Topic，与 `agv_base_driver` 兼容。混流 RX 修复与带宽说明见 **附录 C**。

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
| v2.0-draft.5.6 | 2026-06-16 | 新增附录 C：`rs232_gateway` 混流 RX 修复（旧/新对照）、带宽与 launch 降载 |
| v2.0-draft.5.5 | 2026-06-15 | 新增 §0.2 RS232 / §0.3 CAN 双传输映射与 CAN ID 表 |
| v2.0-draft.5.4 | 2026-06-15 | 修复横排表分隔行列数不一致导致无法渲染 |

---

## 附录 C：Jetson `rs232_gateway` 混流 RX 与带宽（2026-06-16）

### C.1 现象与定责

| 证据 | 说明 |
|------|------|
| F407 `ARB=NORMAL` + `[JETSON BLOB CMD]` | Jetson→MCU 下行 `0x01` 已进仲裁 |
| TimeSync PING 有 RTT | `0xA5` 双向通 |
| `link_test --listen-only --blob-v2` 能收 `0x02/0x03` | MCU PA2 上行格式合法 |
| `launch` 全功能时 topic 空 +「上行超时」 | 同一物理链路，差在 gateway **全双工混流** |

**定责**：MCU RS232 BLOB 阶段可冻结；下一棒在 Jetson `rs232_gateway` 的混流 RX + 上行 watchdog + 115200 带宽管理。

### C.2 115200 带宽估算

8N1 理论峰值：**115200 ÷ 10 ≈ 11520 字节/秒**。

| 方向 | 典型负载（NORMAL） |
|------|-------------------|
| Jetson 下行 `0x01` | 50Hz × 23B ≈ 1150 B/s |
| MCU 上行 `0x02` | 50Hz × 49B ≈ 2450 B/s |
| MCU 上行 `0x03` | 50Hz × 51B ≈ 2550 B/s |
| MCU 上行 `0x04/0x06/0x07/0x08` | 按 §0.4 周期叠加 |
| TimeSync `0xA5` | ~1Hz × 11B（次要） |

双向 BLOB 全速时平均占用可达 **50%～90%** 峰值；**突发**（多帧挤在同一 20ms 窗口）+ USB-TTL 小 FIFO 会导致 ORE/丢字节/解析失步。软件优化与 **降载/提波特率** 需并行。

### C.3 实现状态（`rs232_gateway`）

| 项 | 状态 | 文件 |
|----|------|------|
| TX/RX 线程分离（TX 定时器写，RX 独立线程读） | ✅ 已做 | `rs232_gateway_node.py` |
| 串口写互斥锁 | ✅ 已做 | `serial_io.py` |
| 硬件流控关闭 `rtscts=False` | ✅ 已做 | `serial_io.py` |
| BLOB 混流解析：9B 头提前校验 + 坏头 resync | ✅ 已做 | `service_frame.py` |
| BLOB 模式关闭 V3 `0xAA` 分支 | ✅ 已做 | `Rs232StreamParser(parse_v3=False)` |
| 上行 watchdog 认 BLOB `0x02/0x03/0x04…` | ✅ 已做 | `_last_blob_uplink_time` |
| RX 统计 `blob02/03、svc、hdr_rej`（5s 窗口） | ✅ 已做 | `debug_rx_stats_interval_s` |
| RAW 环形缓冲 + **解析独立线程** | ⏳ 待做 | 当前 RX 线程内仍 `feed()` 解析 |

参考实现（已验证能收上行）：`tools/jetson_rs232_link_test.py` 的 `BlobFrameParser`。

### C.4 错误实现 vs 正确实现（对照）

#### C.4.1 单线程 TX 后立即 RX（旧）

**问题**：50Hz 定时器里 **先 write 再 read+解析**，TX 与 RX 同回调；高负载时读不及时，USB FIFO 溢出。

```python
# 旧：rs232_gateway_node.py — _tick() 单线程读写
def _tick(self) -> None:
    ...
    self._link.write(frame)                    # 发 0x01 + TimeSync
    self._tx_seq = (self._tx_seq + 1) & 0xFF
    raw = self._link.read_available()          # 同线程立刻读
    for item in self._parser.feed(raw):        # 同线程解析
        ...
```

```python
# 新：TX 仍在 _tick()；RX 独立线程 + 队列，主线程只 dispatch
def _tick(self) -> None:
    self._link.write(frame)
    self._process_rx_queue()                   # 从 queue 取已解析帧

def _rx_loop(self) -> None:                     # 独立线程 rs232_rx
    raw = self._link.read_available()
    for item in self._parser.feed(raw):
        self._rx_queue.put(item)
```

#### C.4.2 混流解析：等满帧再验头 / 坏 LEN 堵死（旧）

**问题**：旧 parser 仅 `(self._buf[4]<<8)|self._buf[5]` 当 LEN，**不验** `VER/FRAG/CNT/FLAGS`；非法头用错误 LEN **一直等更多字节**，混流时缓冲区失步。BLOB 模式下仍走 V3 `0xAA` 分支。

```python
# 旧：service_frame.py — Rs232StreamParser.feed()
if magic == BLOB_MAGIC:
    payload_len = (self._buf[4] << 8) | self._buf[5]
    wire_len = BLOB_HDR_LEN + payload_len
    if len(self._buf) < wire_len:
        break                              # 坏 LEN → 永久阻塞
    ...
    if PAYLOAD_LEN.get(msg_id) != payload_len:
        continue                           # 丢帧但不记 hdr_reject
elif magic == FRAME_HEADER:
    ...                                    # BLOB 模式仍解析 V3
else:
    self._buf.pop(0)
```

```python
# 新：凑齐 9B 即 validate_rs232_blob_header()；失败 pop(1) resync
def validate_rs232_blob_header(hdr: bytes) -> tuple[int, int] | None:
    if hdr[6] != 0 or hdr[7] != 1 or hdr[8] != 0:   # RS232 固定 FRAG
        return None
    if PAYLOAD_LEN.get(hdr[2]) != plen:
        return None
    return msg_id, plen

# Rs232StreamParser(parse_v3=not use_blob_v2)  # BLOB 模式不抢 0xAA
validated = validate_rs232_blob_header(hdr)
if validated is None:
    self._buf.pop(0)
    self.stats.hdr_reject += 1
    continue
```

#### C.4.3 上行 watchdog 只认 V3 / 与 TimeSync 混用（旧）

**问题**：`_last_uplink_time` 在 V3 `0x02/0x03` 或 BLOB 发布时混刷；BLOB 模式下 TimeSync 通但 **未收到 BLOB 上行** 时，watchdog 行为与 topic 不一致。

```python
# 旧
def _check_uplink_stale(self) -> None:
    if self._last_uplink_time <= 0:
        return
    if time.monotonic() - self._last_uplink_time > self._uplink_timeout_s:
        self.get_logger().warn("上行超时 ... 0x02/0x03")  # BLOB 模式文案不准
```

```python
# 新：BLOB 模式单独跟踪 MCU 上行 MSG
if msg_id in UPLINK_BLOB_MSG_IDS:
    self._last_blob_uplink_time = time.monotonic()

def _check_uplink_stale(self) -> None:
    last = self._last_blob_uplink_time if self._use_blob_v2 else self._last_uplink_time
    ...
    self.get_logger().warn("上行超时 ... BLOB 0x02/0x03/0x04")
```

#### C.4.4 串口未关硬件流控 / 无写锁（旧）

**问题**：未接 RTS/CTS 时若开启 `CRTSCTS` 会导致假性阻塞；多线程 write 可能交错。

```python
# 旧：serial_io.py
self._ser = serial.Serial(port, baudrate=baud, timeout=0.0)
def write(self, data: bytes) -> None:
    self._ser.write(data)
```

```python
# 新
self._ser = serial.Serial(
    ...,
    timeout=0.0,
    dsrdtr=False,
    rtscts=False,              # 必须关硬件流控
)
self._write_lock = threading.Lock()
def write(self, data: bytes) -> None:
    with self._write_lock:
        self._ser.write(data)
```

### C.5 临时降载（验证带宽 / 混流）

修代码前后均可用：

```bash
pkill -f rs232_gateway; pkill -f agv_base_driver; sleep 1

# 仅验证 MCU 上行
python3 tools/jetson_rs232_link_test.py \
  --port /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0 \
  --listen-only --blob-v2 --time 10

# 减轻混流后再 launch
ros2 launch agv_base_driver jetson_rs232_bringup.launch.py \
  time_sync_enable:=false \
  tx_rate_hz:=20 \
  uplink_timeout_ms:=1000
```

| 参数 | 作用 |
|------|------|
| `time_sync_enable:=false` | 去掉 `0xA5` 混流 |
| `tx_rate_hz:=20` | 下行约 460B/s，心跳仍满足 MCU 300ms |
| `uplink_timeout_ms:=1000` | 联调期减少误报 |

验证：`ros2 topic hz /jetson_rs232/v3_status`；gateway 日志 `RX +5s: blob02=… blob03=…`。

### C.6 联调成功标志

| 侧 | 标志 |
|----|------|
| F407 | `[JETSON BLOB CMD]`，`ARB=NORMAL` |
| Jetson | `/jetson_rs232/v3_status` 有频率，`safety_state=1` |
| gateway | `RX +5s` 中 `blob02>0` 且 `blob03>0`，无持续「上行超时」 |

### C.7 后续待做（Jetson）

1. **RAW RingBuffer + 解析分线程**：RX 线程只 `read()` 入队，解析线程 `feed()`（进一步降低 FIFO 溢出风险）。
2. **可选**：波特率 **230400**（MCU 同步）或 MCU 按 §0.4 降低 `0x06/0x07/0x08` 频率。
3. **不建议优先**：双 fd open 同一 tty、CPU SCHED_FIFO（USB-TTL 收益有限）。

