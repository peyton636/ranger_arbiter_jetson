# BLOB MSG 0x03 mcu_status_t
std_msgs/Header header
uint32 mcu_timestamp_ms
uint8 blob_seq
uint8 seq
uint8 safety_state
uint8 link_state
uint8 limit_factor
int16 arb_v_mm_s
int16 arb_omega_millirad_s
int16 arb_steer_millirad
uint16[4] sonar_mm
uint32[4] sonar_stamp_ms
uint16 nearest_mm
uint8 jetson_seq
