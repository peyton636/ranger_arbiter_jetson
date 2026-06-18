# MCU -> Jetson，BLOB MSG 0x06 / 0x07，agv_motor04_t / agv_motor58_t（44 B）
# Topic：/jetson_eth/blob/motor_04（0x06）、/jetson_eth/blob/motor_58（0x07）
std_msgs/Header header

uint32 mcu_timestamp_ms           # MCU 帧时刻，单位 ms
uint8 blob_seq                     # BLOB SEQ
uint8 motor_base_index             # 本组电机起始编号：0x06->0，0x07->4
MotorCompact[4] motors             # 4 路电机紧凑状态
