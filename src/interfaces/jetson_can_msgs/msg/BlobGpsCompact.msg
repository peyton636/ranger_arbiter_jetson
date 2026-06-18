# BLOB MSG 0x05 gps_compact_t
std_msgs/Header header
uint32 mcu_timestamp_ms
uint8 blob_seq
uint8 flags
uint8 num_sv
uint16 HDOP_INVALID=65535
uint16 hdop_x100
uint16 speed_cm_s
int32 lat_e7
int32 lon_e7
int16 heading_x100
int16 ALT_INVALID=32767
int16 alt_dm
uint32 utc_sec
