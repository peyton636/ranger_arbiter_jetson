# BLOB MSG 0x06 / 0x07 agv_motor04_t / agv_motor58_t
std_msgs/Header header
uint32 mcu_timestamp_ms
uint8 blob_seq
uint8 motor_base_index
MotorCompact[4] motors
