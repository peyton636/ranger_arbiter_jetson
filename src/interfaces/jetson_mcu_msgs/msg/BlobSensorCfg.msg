# Jetson -> MCU，BLOB MSG 0x10，sensor_cfg_t（8 B PAYLOAD，下行）
# Topic（订阅）：/jetson_eth/blob/sensor_cfg、/jetson_rs232/blob/sensor_cfg
std_msgs/Header header

uint32 timestamp_ms               # 配置帧时刻，单位 ms
uint16 threshold_mm                # 避障阈值，单位 mm
uint8 enable_mask                  # 四路超声使能：bit0~3 对应前/后/左/右
