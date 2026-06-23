# 四向超声（TimeSync 对齐 Jetson 时间轴）
# Topic：/jetson_eth/sonar_stamped、/jetson_rs232/sonar_stamped
std_msgs/Header header

uint16 SONAR_INVALID=65535
uint16 front_mm
uint16 back_mm
uint16 left_mm
uint16 right_mm
uint32[4] mcu_stamp_ms              # MCU 各通道原始 tick
uint32[4] jetson_mono_ms            # mcu_stamp - offset_ms
bool time_sync_valid
uint8 source                         # 0=mcu_status(0x03) 1=sensor_blob(0x04)
uint32 mcu_frame_timestamp_ms        # BLOB 帧头 mcu_timestamp_ms
uint8 blob_seq
