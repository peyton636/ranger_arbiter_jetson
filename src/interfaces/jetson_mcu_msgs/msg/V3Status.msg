# STM32 -> Jetson，CAN ID 0x102，V3 上行状态帧（type=0x02，24 字节组帧后）
# Topic：/jetson_eth/v3_status、/jetson_rs232/v3_status、/jetson_can/v3_status
std_msgs/Header header

uint8 seq                         # STM32 发送序号

# 仲裁安全状态 SAFETY_STATE
uint8 SAFETY_NORMAL=1             # 正常
uint8 SAFETY_SPEED_LIMIT=2        # 限速避障
uint8 SAFETY_DEGRADED=3           # 降级/恢复中
uint8 SAFETY_EMERGENCY=4          # 紧急停车
uint8 safety_state

# 链路状态 LINK_STATE 位域：bit0=Jetson心跳丢失 bit1=底盘故障 bit2=蜂鸣器激活
uint8 link_state

uint8 limit_factor                # 限速比例 0~100（%）

int16 fb_v_mm_s                   # 底盘线速度反馈，单位 mm/s
int16 fb_omega_millirad_s         # 自旋角速度反馈，单位 0.001 rad/s
int16 fb_steer_millirad           # 朝向/内转角反馈，单位 0.001 rad

uint16 SONAR_INVALID=65535        # 超声无效值 0xFFFF
uint16 sonar_front_mm             # 前向测距，单位 mm
uint16 sonar_back_mm              # 后向测距，单位 mm
uint16 sonar_left_mm              # 左向测距，单位 mm
uint16 sonar_right_mm             # 右向测距，单位 mm

uint16 battery_voltage_0p1v       # 电池电压，单位 0.1 V
uint8 battery_soc                 # 电池电量，单位 %
