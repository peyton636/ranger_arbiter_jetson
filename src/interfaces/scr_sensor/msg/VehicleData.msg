# AGV 车体综合状态，由 agv_base_driver_node 发布
# 数据来源：V3Status + V3ExtStatus 在 Jetson 侧融合
# Topic：/vehicle/vehicle_data
std_msgs/Header header

# --- 运动反馈（来自 V3Status 0x102）---
int16 linear_velocity_mm_s        # 线速度，单位 mm/s
int16 angular_velocity_millirad_s # 角速度，单位 0.001 rad/s
int16 steer_millirad              # 转角，单位 0.001 rad

# --- 安全/仲裁（来自 V3Status 0x102）---
uint8 SAFETY_NORMAL=1             # 正常
uint8 SAFETY_SPEED_LIMIT=2        # 限速避障
uint8 SAFETY_DEGRADED=3           # 降级/恢复中
uint8 SAFETY_EMERGENCY=4          # 紧急停车
uint8 safety_state

uint8 limit_factor                # 限速比例 0~100（%）

# 链路状态：bit0=Jetson心跳丢失 bit1=底盘故障 bit2=蜂鸣器激活
uint8 link_state

# --- 四向超声测距（来自 V3Status 0x102）---
uint16 SONAR_INVALID=65535        # 无效值 0xFFFF
uint16 sonar_front_mm             # 前，单位 mm
uint16 sonar_back_mm              # 后，单位 mm
uint16 sonar_left_mm              # 左，单位 mm
uint16 sonar_right_mm             # 右，单位 mm

# --- 电池（来自 V3Status 0x102）---
uint16 battery_voltage_0p1v       # 电压，单位 0.1 V
uint8 battery_soc_percent         # 电量，单位 %

# --- 底盘扩展（来自 V3ExtStatus 0x103 最新一帧）---
int16 wheel_rf                    # 右前轮速
int16 wheel_rr                    # 右后轮速
int16 wheel_lr                    # 左后轮速
int16 wheel_lf                    # 左前轮速
int8 motor_temp_max_c             # 电机最高温度，单位 ℃
uint8 driver_state_or             # 驱动器状态按位 OR

# --- 数据 freshness ---
uint8 v3_status_seq               # 对应 V3Status 序号
uint8 v3_ext_status_seq            # 对应 V3ExtStatus 序号
bool data_valid                   # 超时窗口内是否收到有效状态
