# BLOB MSG 0x02 agv_motion_t
std_msgs/Header header
uint32 mcu_timestamp_ms
uint8 blob_seq
uint8 system_info
uint8 motion_info
uint8 light_pack
uint32 fault_code
uint16 battery_voltage_0p1v
int16 linear_velocity
int16 angular_velocity
int16 steering_angle
int16[4] wheel_angle
int16[4] wheel_speed
