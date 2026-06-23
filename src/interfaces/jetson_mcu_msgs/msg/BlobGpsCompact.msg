# MCU -> Jetson，BLOB MSG 0x05，gps_compact_t（32 B PAYLOAD）
# Topic：/jetson_eth/blob/gps、/jetson_rs232/blob/gps
std_msgs/Header header

uint32 mcu_timestamp_ms           # MCU 帧时刻，单位 ms
uint8 blob_seq                     # BLOB SEQ
# flags：bit0=位置有效 bit1=速度有效 bit2=航向有效 bit3=fix有效 bit4=NMEA有效
uint8 flags
uint8 num_sv                         # 卫星数
uint16 HDOP_INVALID=65535          # HDOP 无效值 0xFFFF
uint16 hdop_x100                     # 水平精度因子 ×100
uint16 speed_cm_s                    # 对地速度，单位 cm/s
int32 lat_e7                         # 纬度 × 1e7（度）
int32 lon_e7                         # 经度 × 1e7（度）
int16 heading_x100                   # 航向 ×100（度）
int16 ALT_INVALID=32767              # 海拔无效值 0x7FFF
int16 alt_dm                         # 海拔，单位分米（m = alt_dm / 10）
uint32 utc_sec                       # GPS UTC 秒；无 GPS 时为 0
