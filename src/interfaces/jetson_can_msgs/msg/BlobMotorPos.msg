# MCU -> Jetson，BLOB MSG 0x0B，agv_motor_pos_t（36 B PAYLOAD）
# Topic：/jetson_eth/blob/motor_pos、/jetson_rs232/blob/motor_pos
# 周期约 1 Hz，8 路电机脉冲位置
std_msgs/Header header

uint32 mcu_timestamp_ms           # MCU 帧时刻，单位 ms
uint8 blob_seq                     # BLOB SEQ
int32[8] motor_position            # 8 路电机位置 [0..7 号电机]
