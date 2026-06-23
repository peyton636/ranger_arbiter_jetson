# MCU -> Jetson，BLOB MSG 0x02，agv_motion_t（40 B PAYLOAD）
# Topic：/jetson_eth/blob/motion、/jetson_rs232/blob/motion
std_msgs/Header header

uint32 mcu_timestamp_ms           # MCU 帧时刻，单位 ms（struct byte 0-3，大端）
uint8 blob_seq                     # BLOB 线格式 SEQ 字段
uint8 system_info                  # 底盘系统状态（0x211 system_status）
uint8 motion_info                  # 运动模式回馈（0x291 motion_model）
uint8 light_pack                   # 灯光状态打包（下行 light_info 回馈）
uint32 fault_code                  # 底盘故障码（0x211 FAULT_INFO）
uint16 battery_voltage_0p1v        # 电池电压，单位 0.1 V
int16 linear_velocity              # 线速度反馈，单位 mm/s
int16 angular_velocity             # 角速度反馈，单位 0.001 rad/s
int16 steering_angle               # 转向角反馈，单位 0.001 rad
int16[4] wheel_angle               # 四轮转角 [RF, RR, LR, LF]，单位 0.001 rad
int16[4] wheel_speed               # 四轮轮速 [RF, RR, LR, LF]，固件原始单位
