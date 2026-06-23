# MCU -> Jetson，BLOB MSG 0x04，sensor_blob_t，28 B PAYLOAD
# Topic：/jetson_eth/blob/sensor、/jetson_rs232/blob/sensor
std_msgs/Header header

uint32 mcu_timestamp_ms           # MCU 帧时刻，单位 ms
uint8 blob_seq                     # BLOB SEQ
uint16 SONAR_INVALID=65535         # 超声无效值 0xFFFF
uint16[4] dist_mm                  # 四向测距 [前, 后, 左, 右]，单位 mm
uint32[4] stamp_ms                 # 各路 dist 最后变化 tick（MCU）
uint32[4] jetson_mono_ms           # stamp_ms 换算到 Jetson monotonic ms；无效为 0
bool time_sync_valid                 # offset 有效时 jetson_mono_ms 可用
