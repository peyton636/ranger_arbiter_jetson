# BLOB MSG 0x04 sensor_blob_t
std_msgs/Header header
uint32 mcu_timestamp_ms
uint8 blob_seq
uint16 SONAR_INVALID=65535
uint16[4] dist_mm
uint32[4] stamp_ms
