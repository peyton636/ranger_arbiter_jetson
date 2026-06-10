# STM32B 固件 ↔ PROTOCOL_V3 / Jetson 对照说明

基于当前 `arbiter.c` + `usart`（USART2 收 Jetson）实现与 `ds_jetson_bridge` v0.2 的交叉核对。

---

## 已对齐（可直接联调）

| 项目 | STM32B | Jetson `ds_jetson_bridge` |
|------|--------|---------------------------|
| 帧长 | 24B | 24B |
| 帧头 | `0xAA` | `0xAA` |
| 下行 type | `0x01` | `FRAME_TYPE_DOWN` |
| 上行 type | `0x02` / `0x03` | 已解析 `0x02`（`0x03` 可选） |
| XOR | Byte0~22 | 同 |
| v | mm/s，s16 BE | `linear.x * 1000` |
| ω | **0.001 rad/s**，s16 BE | `angular.z * 1000` |
| 心跳 | `seq` 变化刷新 | 每 TX 帧 `seq++` |
| 超时 | 300ms → `heartbeat_lost` | 200ms 本地零速 + 1s recovery |
| CAN 0x111 | mm/s + 0.001rad/s + steer | 文档一致 |
| 上行 safety | 1~4 映射 | `stm32b/safety_state` |
| limit_factor | `USART3_CalcLimitFactor` | `stm32b/limit_factor` |
| 硬件口 | `USART3_Init` 实际 **USART2** PA2/3 | `ttyUSB5` @ 115200 |

**联调命令：**

```bash
source ~/catkin_ws/install/setup.bash
ros2 launch ds_jetson_bridge jetson_bridge.launch.py
ros2 topic echo /stm32b/safety_state
# B 串口应打印 [JETSON RX] aa 01 ...
```

---

## 须在 `arbiter.h` 核对的常量（与 PROTOCOL_V3 §5 一致）

```c
#define JETSON_V3_FRAME_LEN      24
#define JETSON_FRAME_HEADER      0xAA
#define JETSON_FRAME_TYPE_DOWN   0x01
#define JETSON_FRAME_TYPE_UP_ST  0x02
#define JETSON_FRAME_TYPE_UP_EX  0x03

#define ARBITER_HEARTBEAT_TIMEOUT_MS  300
#define ARBITER_RECOVER_STABLE_MS      1000
#define ARBITER_OBSTACLE_NEAR_MM       300   /* <30cm 紧急 */
#define ARBITER_OBSTACLE_FAR_MM        800   /* <80cm 限速 */
#define ARBITER_MAX_SPEED_MM_S         2000
```

若 `arbiter.h` 仍为 **80 / 150**（mm），与 V3 文档和 `limit_factor` 公式不一致，必须改成 **300 / 800**。

`Arbiter_ParseJetsonCmd` 使用 `JETSON_FRAME_LEN` 时须 **= 24**（与 `JETSON_V3_FRAME_LEN` 同名或 `#define` 统一）。

---

## 建议修改的固件问题（按优先级）

### P0 — NORMAL 模式未透传 `steer_cmd`

```c
// Arbiter_ProcessNormalMode 当前：
arb_state.output.steering = 0;   // ← 忽略 Jetson byte8~9
```

应改为：

```c
arb_state.output.steering = arb_state.jetson_cmd.steer;
```

否则 V3 下行 `steer_cmd` 与 teleop 扩展无效，阿克曼仅靠 `omega` 自旋。

---

### P1 — 降级行为与文档表差异（可接受但需知悉）

| PROTOCOL_V3 §5 | 当前固件 |
|----------------|----------|
| 心跳丢失 + 远距 → v=300mm/s 巡游 | `Arbiter_ProcessDegradedMode` → **四向 if-else**（`Arbiter_ProcessDirectionalPolicy`） |
| 心跳丢失 + 中距 → 后退+转向 | 同上，更激进 |

功能上合理（老板要的超声策略），但 **与文档真值表不完全一致**。建议二选一：

- 改固件贴近文档；或  
- 在 `PROTOCOL_V3.md` §5 注明「实现采用四向规则表，见 `Arbiter_ProcessDirectionalPolicy`」。

---

### P1 — 本地策略角速度注释/单位

```c
#define ARB_CMD_TURN_OMEGA  300   // 注释写 0.01rad/s — 过时
```

CAN 使用 **0.001 rad/s**，`300` = **0.3 rad/s**。请改注释为 `0.001 rad/s`，并确认 300 是否过大。

---

### P2 — `Arbiter_ParseWheelAngle` 未按有符号解析

当前 `(data[0]<<8)|data[1]` 为无符号；应改为 `Chassis_ParseS16BE`。

---

### P2 — 函数命名

`USART3_Init` / `USART3_ProcessRxByte` 实际操作 **USART2**。建议注释标明「历史命名，硬件=USART2」，避免与超声 USART3 混淆。

---

### P2 — `Arbiter_ParseJetsonCmd` 返回值

注释 `0=成功, 1=失败` 与常见习惯一致；`main` 中须 **仅在返回 0 时** 认为本帧有效（若写反会误解析）。

---

## 主循环必备调用顺序（检查清单）

```c
void loop_20ms(void)
{
    u8 jetson_frame[JETSON_V3_FRAME_LEN];

    /* 1. 收 Jetson */
    if(USART3_GetJetsonFrame(jetson_frame))
        Arbiter_ParseJetsonCmd(jetson_frame, JETSON_V3_FRAME_LEN);

    /* 2. 超声 → 四向距离 */
    Arbiter_SetObstacleDistances(front, back, left, right);

    /* 3. CAN 反馈 */
    Arbiter_ProcessCANFeedback();

    /* 4. 仲裁 + 发底盘 */
    Arbiter_Process();
    Arbiter_SendToSTM32A();

    /* 5. 上行 Jetson */
    USART3_SendV3StatusFrame(&arb_state, front, back, left, right);
    USART3_SendV3DetailFrame(&arb_state);   /* 可选 20ms 交替 */
}
```

缺任何一步会导致：能收 `[JETSON RX]` 但无 `0x02`、或底盘不动。

---

## 上行 `v_actual` 单位

`USART3_SendV3StatusFrame` 填入 `motion_fb.linear_speed`（来自 0x221，单位 **0.001 m/s**）。

数值上 **raw = mm/s**（例：0.15 m/s → 150），与 Jetson `stm32b/motion_actual.linear.x = v_mm_s/1000` **一致**，无需再乘换算系数。

---

## Jetson 侧（已完成）

- 24B 下行、`seq`、mode_req 前 50 帧保持 CAN 模式  
- 解析 `0x02` → `stm32b/*` 话题  
- 与本文 P0/P1 无关的固件修完后，teleop 即可在 NORMAL 下透传 v/ω  

`0x03` 扩展帧解析可在 Jetson 下一版加（固件已在 `USART3_SendV3DetailFrame` 发送）。
