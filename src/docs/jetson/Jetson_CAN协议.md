# Jetson 与 STM32B CAN 协议（完整自包含）


| 元数据      | 值                                         |
| -------- | ----------------------------------------- |
| **协议版本** | **v1.4（定稿）**                              |
| **文档日期** | 2026-05-28                                |
| **状态**   | 0x101~~0x10B **已实现**；0x10C~~0x110 **规划中** |


本文档包含 Jetson 与 STM32B（麒麟 F407 仲裁板）通信所需的全部 CAN 协议定义，不依赖跳转其他协议文档。

---

## 1. 总体说明


| 项目     | 说明                                                          |
| ------ | ----------------------------------------------------------- |
| 物理总线   | Jetson 专用 CAN2（PB5=RX, PB6=TX），与底盘 CAN1 隔离                  |
| 默认波特率  | 500 kbps（Classic CAN 2.0B）                                  |
| 应用层统一帧 | V3 24 字节（Header + Type + Seq + Payload + XOR）               |
| V3 校验  | `xor = frame[0] ^ frame[1] ... ^ frame[22]`，应等于 `frame[23]` |
| 字节序    | 大端（BE）                                                      |
| 编译开关   | `JETSON_LINK_CAN=1` 使用 CAN2；`0` 使用 USART2                   |


> 说明：协议目标是 CAN FD 单帧 24B；当前 F407 由于 bxCAN 限制，实际采用 Classic 3x8 分片透传同一 V3 载荷语义。

---

## 2. CAN ID 分配


| CAN ID        | 方向              | 内容                  | 当前实现        |
| ------------- | --------------- | ------------------- | ----------- |
| `0x101`       | Jetson -> STM32 | V3 下行控制帧（type=0x01） | 已实现（3x8 组帧） |
| `0x102`       | STM32 -> Jetson | V3 上行状态帧（type=0x02） | 已实现         |
| `0x103`       | STM32 -> Jetson | V3 上行扩展帧（type=0x03） | 已实现         |
| `0x104`       | STM32 -> Jetson | GPS 帧 A             | 已实现         |
| `0x105`       | STM32 -> Jetson | GPS 帧 B             | 已实现         |
| `0x106`       | STM32 -> Jetson | GPS 帧 C             | 已实现         |
| `0x107`       | Jetson -> STM32 | 时间同步请求              | 已实现         |
| `0x108`       | STM32 -> Jetson | 时间同步响应              | 已实现         |
| `0x109`       | STM32 -> Jetson | 故障码上报               | 已实现         |
| `0x10A`       | Jetson -> STM32 | 状态查询请求              | 已实现         |
| `0x10B`       | STM32 -> Jetson | 状态查询响应（快照）          | 已实现         |
| `0x10C~0x10E` | 双向              | 远程升级（OTA）           | 规划中         |
| `0x10F`       | STM32 -> Jetson | IMU 数据（预留）          | 规划中         |
| `0x110`       | Jetson -> STM32 | 虚拟路标/临时目标点          | 规划中         |


---

## 3. 传输层封装（当前 F407 实现）

### 3.1 V3 帧（0x101/0x102/0x103）

- 应用层长度固定 24 字节；
- Classic CAN 下拆为 3 帧发送，每帧 8 字节，按顺序拼接：
  - frag0 = `v3[0..7]`
  - frag1 = `v3[8..15]`
  - frag2 = `v3[16..23]`
- 接收端必须按同 CAN ID 顺序重组后再做 V3 校验。

### 3.2 GPS 帧（0x104 / 0x105 / 0x106）— 方式 B：不同 CAN ID

GPS **不用同一 ID 组帧**，而是用 **三个独立 CAN ID** 各发 8 字节，Jetson 按 ID 直接解析，**无需缓存/序号重组**：


| CAN ID    | 内容                            | byte1（FRAG_IDX）    |
| --------- | ----------------------------- | ------------------ |
| **0x104** | flags + num_sv + hdop + speed | 固定 `0`（仅作校验，非组帧依据） |
| **0x105** | lat + heading                 | 固定 `1`             |
| **0x106** | lon + alt                     | 固定 `2`             |


- 每帧 byte0 = 魔数 `0xA4`；**不复用 V3 XOR**。
- 某一帧丢失不影响其余帧；周期内收到几个算几个。
- 发送顺序建议：`0x104 -> 0x105 -> 0x106`，帧间间隔 2ms。

---

## 4. V3 下行控制帧（type=0x01，CAN ID 0x101）

### 4.1 字段定义（24B）


| 字节    | 字段           | 类型  | 说明          |
| ----- | ------------ | --- | ----------- |
| 0     | HEADER       | u8  | 固定 `0xAA`   |
| 1     | FRAME_TYPE   | u8  | 固定 `0x01`   |
| 2     | SEQ          | u8  | Jetson 发送序号 |
| 3     | MODE_REQ     | u8  | 控制模式请求      |
| 4     | V_H          | u8  | 线速度高字节      |
| 5     | V_L          | u8  | 线速度低字节      |
| 6     | OMEGA_H      | u8  | 自旋速度高字节     |
| 7     | OMEGA_L      | u8  | 自旋速度低字节     |
| 8     | STEER_H      | u8  | 转角高字节       |
| 9     | STEER_L      | u8  | 转角低字节       |
| 10    | MOTION_MODEL | u8  | 运动模式        |
| 11    | LIGHT_ENABLE | u8  | 灯光使能        |
| 12    | LIGHT_MODE   | u8  | 灯光模式        |
| 13    | CLEAR_ERROR  | u8  | 清错请求        |
| 14~22 | RSV          | u8  | 保留，发 0      |
| 23    | XOR_CHK      | u8  | XOR 校验      |


### 4.2 组合字段


| 信号    | 公式       | 类型  | 单位  |
| ----- | -------- | --- | --- |
| V     | `(b4<<8) | b5` | s16 |
| OMEGA | `(b6<<8) | b7` | s16 |
| STEER | `(b8<<8) | b9` | s16 |


### 4.3 枚举值

#### MODE_REQ


| 值      | 含义            |
| ------ | ------------- |
| `0x00` | 待机            |
| `0x01` | CAN 指令控制      |
| `0x02` | 遥控请求（当前按待机处理） |


#### MOTION_MODEL


| 值      | 含义    |
| ------ | ----- |
| `0x00` | 前后阿克曼 |
| `0x01` | 斜移    |
| `0x02` | 自旋    |
| `0x03` | 驻车    |


---

## 5. V3 上行状态帧（type=0x02，CAN ID 0x102）

### 5.1 字段定义（24B）


| 字节  | 字段           | 类型  | 说明          |
| --- | ------------ | --- | ----------- |
| 0   | HEADER       | u8  | 固定 `0xAA`   |
| 1   | FRAME_TYPE   | u8  | 固定 `0x02`   |
| 2   | SEQ          | u8  | STM32 发送序号  |
| 3   | SAFETY_STATE | u8  | 仲裁安全状态      |
| 4   | LINK_STATE   | u8  | 链路/系统状态位    |
| 5   | LIMIT_FACTOR | u8  | 限速比例（0~100） |
| 6   | FB_V_H       | u8  | 底盘线速度高字节    |
| 7   | FB_V_L       | u8  | 底盘线速度低字节    |
| 8   | FB_OMEGA_H   | u8  | 自旋速度高字节     |
| 9   | FB_OMEGA_L   | u8  | 自旋速度低字节     |
| 10  | FB_STEER_H   | u8  | 朝向/内转角高字节   |
| 11  | FB_STEER_L   | u8  | 朝向/内转角低字节   |
| 12  | SONAR_F_H    | u8  | 前向测距高字节     |
| 13  | SONAR_F_L    | u8  | 前向测距低字节     |
| 14  | SONAR_B_H    | u8  | 后向测距高字节     |
| 15  | SONAR_B_L    | u8  | 后向测距低字节     |
| 16  | SONAR_L_H    | u8  | 左向测距高字节     |
| 17  | SONAR_L_L    | u8  | 左向测距低字节     |
| 18  | SONAR_R_H    | u8  | 右向测距高字节     |
| 19  | SONAR_R_L    | u8  | 右向测距低字节     |
| 20  | BAT_V_H      | u8  | 电池电压高字节     |
| 21  | BAT_V_L      | u8  | 电池电压低字节     |
| 22  | SOC          | u8  | 电池 SOC(%)   |
| 23  | XOR_CHK      | u8  | XOR 校验      |


### 5.2 组合字段


| 信号            | 公式        | 类型   | 单位  |
| ------------- | --------- | ---- | --- |
| FB_V          | `(b6<<8)  | b7`  | s16 |
| FB_OMEGA      | `(b8<<8)  | b9`  | s16 |
| FB_STEER      | `(b10<<8) | b11` | s16 |
| SONAR_F/B/L/R | `(bH<<8)  | bL`  | u16 |
| BAT_V         | `(b20<<8) | b21` | u16 |


#### SONAR_* 测距字段约定


| 项目    | 约定                                                             |
| ----- | -------------------------------------------------------------- |
| 单位    | **mm（毫米）**                                                     |
| 数据来源  | `DistSnapshot` 滤波后四向距离（前/后/左/右）                                |
| 无效值   | `**0xFFFF`**（`DS_DIST_UNKNOWN`，表示无数据/超时/未更新）                   |
| 有效范围  | **0 ~ 60000 mm**（与测距模组量程一致；`>=60000` 的模组哨兵值在 STM32 侧已滤为无效）     |
| 障碍物判断 | 结合 `SAFETY_STATE`、`LIMIT_FACTOR` 与四向 SONAR；单路 `0xFFFF` 表示该方向未知 |


### 5.3 SAFETY_STATE


| 值      | 模式                    |
| ------ | --------------------- |
| `0x01` | NORMAL                |
| `0x02` | SPEED_LIMIT           |
| `0x03` | DEGRADED / RECOVERING |
| `0x04` | EMERGENCY             |


### 5.4 LINK_STATE 位定义（已实现）


| Bit | 名称               | =1 含义                           |
| --- | ---------------- | ------------------------------- |
| 0   | `JETSON_HB_LOST` | Jetson 心跳超时（>300ms）             |
| 1   | `CHASSIS_FAULT`  | 底盘系统异常（system_status=0x02）      |
| 2   | `BEEP_ACTIVE`    | 测距报警蜂鸣器处于激活（`BEEP_GetDuty()>0`） |
| 3~7 | RSV              | 保留，当前发 0                        |


### 5.5 LIMIT_FACTOR 计算

- 非 `SPEED_LIMIT`：固定 `100`
- `SPEED_LIMIT` 下按最近障碍距离线性计算：
  - `dist <= near` -> `0`
  - `dist >= far` -> `100`
  - 其间线性插值
- 当前参数：`near=60mm`, `far=180mm`

---

## 6. V3 上行扩展帧（type=0x03，CAN ID 0x103）

### 6.1 字段定义（24B）


| 字节  | 字段              | 类型  | 说明           |
| --- | --------------- | --- | ------------ |
| 0   | HEADER          | u8  | 固定 `0xAA`    |
| 1   | FRAME_TYPE      | u8  | 固定 `0x03`    |
| 2   | SEQ             | u8  | STM32 发送序号   |
| 3   | WHEEL_RF_H      | u8  | 右前轮速高字节      |
| 4   | WHEEL_RF_L      | u8  | 右前轮速低字节      |
| 5   | WHEEL_RR_H      | u8  | 右后轮速高字节      |
| 6   | WHEEL_RR_L      | u8  | 右后轮速低字节      |
| 7   | WHEEL_LR_H      | u8  | 左后轮速高字节      |
| 8   | WHEEL_LR_L      | u8  | 左后轮速低字节      |
| 9   | WHEEL_LF_H      | u8  | 左前轮速高字节      |
| 10  | WHEEL_LF_L      | u8  | 左前轮速低字节      |
| 11  | STEER_RF_H      | u8  | 右前转角高字节      |
| 12  | STEER_RF_L      | u8  | 右前转角低字节      |
| 13  | STEER_RR_H      | u8  | 右后转角高字节      |
| 14  | STEER_RR_L      | u8  | 右后转角低字节      |
| 15  | STEER_LR_H      | u8  | 左后转角高字节      |
| 16  | STEER_LR_L      | u8  | 左后转角低字节      |
| 17  | STEER_LF_H      | u8  | 左前转角高字节      |
| 18  | STEER_LF_L      | u8  | 左前转角低字节      |
| 19  | MOTOR_TEMP_MAX  | s8  | 8 路电机最高温度（℃） |
| 20  | DRIVER_STATE_OR | u8  | 8 路驱动状态按位 OR |
| 21  | RSV21           | u8  | 保留 0         |
| 22  | RSV22           | u8  | 保留 0         |
| 23  | XOR_CHK         | u8  | XOR 校验       |


---

## 7. GPS 上行分片（CAN ID 0x104/0x105/0x106）

### 7.1 分片格式

#### 0x104（帧 0，CAN ID 0x104）


| byte | 字段        | 说明                                           |
| ---- | --------- | -------------------------------------------- |
| 0    | MAGIC     | 固定 `0xA4`                                    |
| 1    | FRAG_IDX  | 固定 `0`                                       |
| 2    | FLAGS     | 定位状态位                                        |
| 3    | NUM_SV    | 卫星数                                          |
| 4~5  | HDOP_X100 | HDOP*100，未知为 `0xFFFF`                        |
| 6~7  | SPEED_CMS | 速度 **cm/s**，u16 BE，范围 0~~65535（约 0~~655 m/s） |


#### 0x105（帧 1，CAN ID 0x105）


| byte | 字段           | 说明            |
| ---- | ------------ | ------------- |
| 0    | MAGIC        | 固定 `0xA4`     |
| 1    | FRAG_IDX     | 固定 `1`        |
| 2~5  | LAT_E7       | 纬度 * 1e7（s32） |
| 6~7  | HEADING_X100 | 航向 *100（s16）  |


#### 0x106（帧 2，CAN ID 0x106）


| byte | 字段       | 说明                        |
| ---- | -------- | ------------------------- |
| 0    | MAGIC    | 固定 `0xA4`                 |
| 1    | FRAG_IDX | 固定 `2`（校验用，组帧看 CAN ID）    |
| 2~5  | LON_E7   | 经度 * 1e7（s32 BE）          |
| 6~7  | ALT_DM   | 海拔 **x10（分米）**，s16 BE；见下表 |


#### ALT_DM 海拔约定


| 项目    | 约定                                            |
| ----- | --------------------------------------------- |
| 换算    | 实际海拔(m) = `ALT_DM / 10.0`                     |
| 有效范围  | **-10000 ~ +10000 dm**（-1000m ~ +1000m）       |
| 无效值   | `**0x7FFF`（32767）**，与有效负海拔不冲突                 |
| 无 fix | 发 `0x7FFF` 或 0（bridge 以 `FLAGS.POS_VALID` 为准） |


### 7.2 FLAGS 位定义


| Bit | 名称            | =1 含义            |
| --- | ------------- | ---------------- |
| 0   | POS_VALID     | 位置有效             |
| 1   | VEL_VALID     | 速度有效             |
| 2   | HEADING_VALID | 航向有效             |
| 3   | FIX_3D_LIKE   | 可用卫星和定位状态满足有效定位  |
| 4   | USEFULL       | 兼容 NMEA `A` 有效标志 |
| 5~7 | RSV           | 保留               |


> **时间戳说明**：当前 0x104~0x106 **未携带采集时刻**。融合定位需配合 **§8.1 时间同步（0x107/0x108）**，或由 Jetson 以收到帧的 `ros::Time::now()` 打软时间戳（精度较低）。

---

## 8. 扩展服务帧（0x107 ~ 0x10B，高优先级）

扩展帧 **不复用 V3 魔数 0xAA**，byte0 为子命令/类型；DLC 见各表。均为 **Classic CAN DLC<=8**（与现 F407 实现一致）。

### 8.1 时间同步（0x107 / 0x108）

用于 Jetson 与 STM32 对齐时间基准，给 GPS/IMU 融合提供参考。

#### 0x107 时间同步请求（Jetson -> STM32，DLC=1）


| byte | 字段  | 说明                     |
| ---- | --- | ---------------------- |
| 0    | CMD | 固定 `**0x01`** = 请求时间同步 |


- Jetson 在启动、GPS fix 变化、或每 **10s** 周期性发送（建议）。
- STM32 收到后 **立即** 回复一帧 0x108（不走 V3 周期）。

#### 0x108 时间同步响应（STM32 -> Jetson，DLC=8）


| byte | 字段             | 类型     | 说明                                                                                                 |
| ---- | -------------- | ------ | -------------------------------------------------------------------------------------------------- |
| 0~3  | SYSTEM_TICK_MS | u32 BE | STM32 单调毫秒：`xTaskGetTickCount()*portTICK_PERIOD_MS`                                                |
| 4~7  | UTC_TIME_SEC   | u32 BE | **Unix 时间戳**（自 **1970-01-01 00:00:00 UTC** 起的秒数）；STM32 由 `GPS_GetUtcUnixSec()` 填充；**无有效 GPS 时填 0** |


**UTC 来源（实现建议）**


| 优先级 | 来源                                                                               |
| --- | -------------------------------------------------------------------------------- |
| 1   | `GPS_GetUtcUnixSec()`：由 NMEA/CASIC 的 `date` + `utc_time` 换算为 **Unix 秒（1970 基准）** |
| 2   | 无有效 GPS 时填 **0**，Jetson 仅用 `SYSTEM_TICK_MS` 做相对同步                                |


**Jetson 用法**

```text
t_gps_capture ~= UTC_TIME_SEC + (t_jetson_rx - SYSTEM_TICK_MS 映射到 Jetson 时钟)
```

#### 与 0x104 GPS 的配合


| 方式       | 说明                                            |
| -------- | --------------------------------------------- |
| 推荐       | 0x108 提供时间基准；0x104~0x106 仍只传观测值，融合节点按同步偏差打时间戳 |
| 可选（v1.3） | 在 0x104 FLAGS 之上增加 `GPS_TICK_MS` 专用帧或扩 DLC    |


---

### 8.2 故障上报（0x109，STM32 -> Jetson）

将底盘/本地故障主动推送给 Jetson（当前仅串口打印 `fault_info`）。

#### 0x109 故障快照（上行，DLC=8）


| byte | 字段               | 类型     | 说明                                         |
| ---- | ---------------- | ------ | ------------------------------------------ |
| 0    | FAULT_CODE_1     | u8     | 本协议统一错误码（见 §9）或底盘故障摘要                      |
| 1    | FAULT_CODE_2     | u8     | 第二故障码，无则 `0x00`                            |
| 2    | FAULT_CODE_3     | u8     | 第三故障码                                      |
| 3    | FAULT_CODE_4     | u8     | 第四故障码                                      |
| 4~5  | FAULT_TS_MS      | u16 BE | 故障发生时 `SYSTEM_TICK_MS` 低 16 位（或全 32 位另帧扩展） |
| 6    | CHASSIS_SYS      | u8     | 底盘 0x211 `system_status` 原样                |
| 7    | CHASSIS_FAULT_B0 | u8     | 底盘 0x211 `FAULT_INFO` 最低字节（快速位域）           |


**FAULT_CODE 填充规则（实现建议）**


| 条件                    | FAULT_CODE_1 建议值    |
| --------------------- | ------------------- |
| 无故障                   | `0x00`              |
| Jetson 心跳丢失           | `0x10`（见 §9）        |
| 底盘 system_status=0x02 | `0x20`              |
| 紧急停车 / 四向危险           | `0x30`              |
| 最近距未知                 | `0x40`              |
| 四路测距均无效               | `0x03`              |
| GPS 无 fix             | `0x02`              |
| 底盘 `fault_info!=0`    | `0x21` + 附 byte7 位域 |


- 发送策略：**事件触发**（故障边沿）+ **1 Hz 保活**（有活跃故障时）。
- 完整 32 位 `FAULT_INFO` 见底盘协议 0x211；0x109 为 Jetson 轻量摘要。

---

### 8.3 状态查询（0x10A / 0x10B）

Jetson 主动拉取快照，用于调试、异常恢复、非周期巡检。

#### 0x10A 状态查询请求（Jetson -> STM32，DLC=1）


| byte | 字段    | 说明                    |
| ---- | ----- | --------------------- |
| 0    | QUERY | `**0x01`** = 查询关键状态快照 |


#### 0x10B 状态查询响应（STM32 -> Jetson，DLC=8）

打包当前最关键量（**不等同完整 0x102/0x103**，而是 8B 快速快照）：


| byte | 字段           | 类型     | 说明                    |
| ---- | ------------ | ------ | --------------------- |
| 0    | SAFETY_STATE | u8     | 同 0x102 byte3         |
| 1    | LINK_STATE   | u8     | 同 0x102 byte4         |
| 2~3  | FB_V         | s16 BE | 当前线速度 mm/s            |
| 4~5  | NEAREST_MM   | u16 BE | 最近障碍距离 mm；无效 `0xFFFF` |
| 6    | LIMIT_FACTOR | u8     | 限速比例 0~100            |
| 7    | FLAGS        | u8     | 见下表                   |


**0x10B FLAGS（byte7）**


| Bit | 名称                 | =1 含义                                                               |
| --- | ------------------ | ------------------------------------------------------------------- |
| 0   | GPS_FIX            | `GPS_HasFix()`                                                      |
| 1   | CHASSIS_CAN_ACTIVE | 底盘 0x211 `mode_control == 0x01`（CAN 指令模式）                           |
| 2   | DIST_VALID         | **至少一路**测距有效（`obstacle_valid_mask != 0`，即四向 SONAR 中至少一个不是 `0xFFFF`） |
| 3~7 | RSV                | 0                                                                   |


- STM32 收到 0x10A 后 **50ms 内** 回复 0x10B。
- 若需完整状态，Jetson 仍应解析周期性的 **0x102 + 0x103**。

---

### 8.4 远程升级（0x10C ~ 0x10E，规划中）


| CAN ID | 方向              | DLC | 说明                                                                      |
| ------ | --------------- | --- | ----------------------------------------------------------------------- |
| 0x10C  | Jetson -> STM32 | 2   | `byte0` cmd：`0x01` 开始 / `0x02` 取消；`byte1` total_frames                  |
| 0x10D  | Jetson -> STM32 | 8   | `byte0` frame_idx；`byte1~7` data                                        |
| 0x10E  | STM32 -> Jetson | 2   | `byte0` status：`0x00` OK / `0x01` fail / `0x02` busy；`byte1` progress % |


> 依赖 Bootloader / Flash 分区方案，当前工程 **未实现**。

---

### 8.5 IMU 预留（0x10F，规划中）

当前工程 **未接 IMU**；若未来启用 MPU6050 等，建议：


| CAN ID | DLC        | 内容                                           |
| ------ | ---------- | -------------------------------------------- |
| 0x10F  | 8 或 16（FD） | 加速度 mg（s16 x3）+ 角速度 mdps（s16 x3），各轴 **2 字节** |


原 8 字节装不下三轴陀螺仪（各轴需 s16），**定稿建议 0x10F + 0x10F_AUX 两帧** 或 CAN FD DLC=16。

---

### 8.6 虚拟路标（0x110，规划中）


| byte | 字段       | 说明                      |
| ---- | -------- | ----------------------- |
| 0    | TYPE     | `0x01` 临时停车点；`0x02` 绕行点 |
| 1~2  | X_MM     | s16 BE，车体坐标系 mm         |
| 3~4  | Y_MM     | s16 BE                  |
| 5    | RADIUS_M | u8，有效半径 m               |
| 6~7  | RSV      | 0                       |


> 需在仲裁层增加路标队列与消耗逻辑；**未实现**。

---

## 9. 统一错误码（本协议 + 摘要）

用于 0x109 `FAULT_CODE`_* 与 Jetson 日志。


| 码      | 名称                 | 含义                               | 检测来源                                 |
| ------ | ------------------ | -------------------------------- | ------------------------------------ |
| `0x00` | OK                 | 无错误                              | —                                    |
| `0x02` | GPS_NO_FIX         | GPS 无有效定位                        | `GPS_HasFix()==0`                    |
| `0x03` | DIST_SENSOR_FAULT  | 四路测距均无效或持续超时                     | `DistanceSensor`                     |
| `0x04` | CHASSIS_CAN_FAULT  | 底盘 CAN 无反馈 / 0x211 异常            | CAN1 / `sys_status`                  |
| `0x10` | JETSON_HB_LOST     | Jetson 心跳超时（>300ms 无新 0x101 SEQ） | `heartbeat_lost` / `LINK_STATE` bit0 |
| `0x20` | CHASSIS_SYS_ERR    | 底盘 system_status=0x02            | 0x211 byte0                          |
| `0x21` | CHASSIS_FAULT_INFO | 底盘 FAULT_INFO 非 0                | 0x211 byte4~7                        |
| `0x30` | OBSTACLE_EMERGENCY | 紧急停车 / 四向危险                      | `SAFETY_STATE=EMERGENCY`             |
| `0x40` | DIST_ALL_UNKNOWN   | 最近距未知                            | `nearest_dist=0xFFFF`                |
| `0xFF` | UNKNOWN            | 未分类                              | —                                    |


> **说明**：原草案 `0x01 CAN2_RX_TIMEOUT` 与 `0x10 JETSON_HB_LOST` 语义重复，**v1.4 起废除 0x01**，统一用 `**0x10*`* 表示应用层心跳丢失（非 CAN 物理层错误）。

底盘 **FAULT_INFO** 32 位位域定义见 RANGER MINI 原厂手册 / [硬件连接与通信协议.md](./硬件连接与通信协议.md) §7.1。

---

## 10. 周期建议（当前任务配置）


| 帧               | 周期                      |
| --------------- | ----------------------- |
| 0x102 状态帧       | ~20ms                   |
| 0x103 扩展帧       | ~40ms                   |
| 0x104~0x106 GPS | ~100ms                  |
| 0x101 下行        | Jetson 按控制回路发送（建议 20ms） |
| 0x107 时间同步请求    | 建议 10s 周期或事件触发（已实现）     |
| 0x109 故障上报      | 事件触发 + 有故障时 1Hz（已实现）    |
| 0x10A 状态查询      | 按需（已实现）                 |


---

## 11. 心跳与超时


| 项目              | 值                              | 说明                                                                                                      |
| --------------- | ------------------------------ | ------------------------------------------------------------------------------------------------------- |
| Jetson 心跳超时     | **300 ms**                     | `ARBITER_HEARTBEAT_TIMEOUT_MS`；连续 300ms 未收到 **0x101 新 SEQ**，`LINK_STATE.JETSON_HB_LOST=1`，进入 `DEGRADED` |
| 0x101 建议发送周期    | **20 ms**                      | Jetson 侧控制回路；非强制，但低于 300ms 即可保活                                                                         |
| 底盘 CAN 0x111 超时 | **500 ms**                     | 底盘协议约定（CAN1，与 Jetson 链路无关）；超时底盘自行停车                                                                     |
| 恢复条件            | 收到新 SEQ + **RECOVERING 持续 1s** | `ARBITER_RECOVER_STABLE_MS=1000`；从 `RECOVERING` 回 `NORMAL`（代码在 `arbiter.c`）                             |
| GPS 组帧超时        | 无                              | 0x104/105/106 独立 ID，不做三帧重组                                                                              |


---

## 12. 编译切换（`JETSON_LINK_CAN`）

宏定义位置：`APP/jetson_can/jetson_can.h`

```c
#define JETSON_LINK_CAN  1   /* 1=CAN2，0=USART2 */
```


| `JETSON_LINK_CAN` | 初始化                                        | 下行接收                                             | 上行发送                                                   |
| ----------------- | ------------------------------------------ | ------------------------------------------------ | ------------------------------------------------------ |
| **1**             | `JetsonCAN_Init()` -> `CAN2_Init_Jetson()` | `JetsonCAN_ProcessRx()` + `JetsonCAN_GetFrame()` | `USART3_DeliverV3Frame()` -> `JetsonCAN_SendV3Frame()` |
| **0**             | `USART3_Init()`（PA2/PA3 USART2）            | `USART3_GetJetsonFrame()` + `USART2_IRQHandler`  | `USART3_SendData()` 24B                                |


**受影响文件**


| 文件               | 作用                                    |
| ---------------- | ------------------------------------- |
| `jetson_can.c/h` | CAN2 组帧/拆帧、GPS 发送                     |
| `can2.c/h`       | CAN2 硬件初始化、过滤器                        |
| `usart3.c`       | V3 编解码；`DeliverV3Frame` 分发 CAN/串口     |
| `rtos_tasks.c`   | JetsonTask / GpsTask 选 CAN 或 USART 路径 |
| `app_boot.c`     | 启动时 `JetsonCAN_Init` 或 `USART3_Init`  |


> 当前工程**未**使用独立的 `jetson_link.c`；收发经 `usart3` + `jetson_can` 两套 API 由宏分支。

---

## 13. CAN2 过滤器配置（STM32）

**当前**：接收 Jetson 下行 **0x101 / 0x107 / 0x10A**（Filter 14/15/16）。

实现位置：`APP/can/can2.c` -> `CAN2_Mode_Init()`


| 配置项   | 值                                  |
| ----- | ---------------------------------- |
| 过滤器编号 | **14**（CAN2 专用 bank 14~27）         |
| 模式    | IdMask，32bit scale                 |
| 匹配 ID | `0x101`（标准帧，左移 5 位写入 FilterIdHigh） |
| 掩码    | `0x7FF`（11-bit 全匹配）                |
| FIFO  | `CAN_FIFO0`                        |
| 发送    | 0x102~0x106 由 STM32 主动发出，无需 RX 过滤  |


**实现 0x107/0x10A 后需扩展过滤器**（二选一）：


| 方案  | 做法                                                            |
| --- | ------------------------------------------------------------- |
| A   | 掩码匹配 `0x100~0x10F` 下行请求类（0x101/0x107/0x10A/0x10C/0x10D/0x110） |
| B   | 增加 Filter 15：精确 0x107；Filter 16：精确 0x10A                      |


```c
CAN_FilterInitStructure.CAN_FilterNumber = 14;
CAN_FilterInitStructure.CAN_FilterIdHigh = (JETSON_CAN_ID_DOWN << 5);  /* 0x101 */
CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (0x7FFu << 5);
CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
```

---

## 14. 当前实现状态


| 项目                                           | 状态                             |
| -------------------------------------------- | ------------------------------ |
| 0x101 下行解析（V3 type=0x01）                     | 已实现                            |
| 0x102 状态帧（含 `SAFETY_STATE` / `LIMIT_FACTOR`） | 已实现                            |
| 0x102 `LINK_STATE bit2=BEEP_ACTIVE`          | 已实现                            |
| 0x103 扩展帧（轮速/转角/温度/驱动状态）                     | 已实现                            |
| 0x104~0x106 GPS 三 ID                         | 已实现                            |
| 0x107/0x108 时间同步                             | **已实现**（`GPS_GetUtcUnixSec()`） |
| 0x109 故障上报                                   | **已实现**（事件 + 1Hz 保活）           |
| 0x10A/0x10B 状态查询                             | **已实现**                        |
| 0x10C~0x10E OTA                              | 规划中                            |
| 0x10F IMU                                    | 规划中（无硬件）                       |
| 0x110 虚拟路标                                   | 规划中                            |
| CAN FD 单帧 24B                                | 预留（F407 bxCAN 不支持 FD）          |


**Jetson 侧待做**

1. 订阅 0x108/0x109/0x10B，发 0x107/0x10A 测试
2. 用 0x108 给 GPS/IMU 打时间戳

---

## 15. 变更记录


| 版本       | 日期             | 修改内容                                                      |
| -------- | -------------- | --------------------------------------------------------- |
| v1.0     | 2026-05-28     | 初版：0x101~0x106，V3 + GPS                                   |
| v1.1     | 2026-05-28     | 审查修订：GPS 方式 B、SONAR/ALT、心跳/编译/过滤器                         |
| v1.2     | 2026-05-28     | 扩展 0x107~0x10B 协议、错误码表、OTA/IMU/路标规划                       |
| v1.3     | 2026-05-28     | 0x107~0x10B 代码实现；CAN2 Filter 15/16；`GPS_GetUtcUnixSec()`  |
| **v1.4** | **2026-05-28** | **定稿修订**：Unix 时间基准、废除错误码 0x01、0x10B FLAGS 定义；附录负载/时序/解析示例 |


---

## 附录 A：CAN2 总线负载估算（500 kbps）

Classic CAN 标准帧 8 字节数据，含位填充与帧间隔，按约 **110 bit/帧** 估算。


| 帧                     | 周期     | 每周期 bit 数 | 平均负载         |
| --------------------- | ------ | --------- | ------------ |
| 0x102                 | 20 ms  | ~110      | ~5.5 kbps    |
| 0x103                 | 40 ms  | ~110      | ~2.75 kbps   |
| 0x104~0x106           | 100 ms | ~330（3 帧） | ~3.3 kbps    |
| 0x101（Jetson 发）       | 20 ms  | ~110      | ~5.5 kbps    |
| 0x108 / 0x10B / 0x109 | 按需     | 可忽略       | <0.5 kbps    |
| **合计（典型）**            |        |           | **~17 kbps** |


结论：相对 **500 kbps** 仲裁段，负载 **<5%**，余量充足。

---

## 附录 B：典型交互时序

```text
Jetson                               STM32
   |                                   |
   |------ 0x101 (V3 下行, 3x8) ------>|
   |                                   |
   |<----- 0x102 (状态帧, 3x8) ---------|  ~20ms 交替
   |<----- 0x103 (扩展帧, 3x8) ---------|
   |<----- 0x104 / 0x105 / 0x106 ------|  ~100ms GPS
   |                                   |
   |------ 0x107 (时间同步, 01) ------->|
   |<----- 0x108 (tick + Unix UTC) -----|
   |                                   |
   |------ 0x10A (状态查询, 01) ------->|
   |<----- 0x10B (8B 快照) -------------|
   |                                   |
   |<----- 0x109 (故障, 事件/1Hz) ------|  有故障时
```

---

## 附录 C：Jetson 解析示例（Python）

```python
def u16be(b, i):
    return (b[i] << 8) | b[i + 1]

def s16be(b, i):
    v = u16be(b, i)
    return v - 0x10000 if v >= 0x8000 else v

def u32be(b, i):
    return (b[i] << 24) | (b[i+1] << 16) | (b[i+2] << 8) | b[i+3]

def parse_v3_status(data):  # 0x102 拼满 24B 后
    return {
        "safety": data[3],
        "link": data[4],
        "limit_pct": data[5],
        "v_mm_s": s16be(data, 6),
        "sonar_f_mm": u16be(data, 12),
    }

def parse_gps_a(data):  # CAN ID 0x104, 8B
    if data[0] != 0xA4:
        return None
    return {"flags": data[2], "num_sv": data[3],
            "hdop": u16be(data, 4) / 100.0,
            "speed_m_s": u16be(data, 6) / 100.0}

def parse_time_rsp(data):  # 0x108
    return {"tick_ms": u32be(data, 0),
            "utc_unix": u32be(data, 4)}  # 1970 基准，0=无 GPS

def parse_status_snap(data):  # 0x10B
    return {
        "safety": data[0], "link": data[1],
        "v_mm_s": s16be(data, 2),
        "nearest_mm": u16be(data, 4),
        "limit_pct": data[6],
        "gps_fix": bool(data[7] & 1),
        "dist_valid": bool(data[7] & 4),
    }
```

> V3 帧（0x101/102/103）需先将同 ID 的 3 片 8B 拼成 24B，再校验 `data[0]==0xAA` 与 XOR。

