# MCU -> Jetson，BLOB MSG 0x08，agv_energy_t（41 B PAYLOAD）
# Topic：/jetson_eth/blob/energy、/jetson_rs232/blob/energy
std_msgs/Header header

uint32 mcu_timestamp_ms           # MCU 帧时刻，单位 ms
uint8 blob_seq                     # BLOB SEQ
int32[4] odom                       # 四轮里程计脉冲/计数 [RF, RR, LR, LF]
uint8 bms_soc                        # BMS 电量，单位 %
uint8 bms_soh                        # BMS 健康度，单位 %
uint16 bms_voltage_0p1v              # BMS 电压，单位 0.1 V
int16 bms_current                    # BMS 电流
int16 bms_temperature                # BMS 温度
uint8 bms_alarm1                     # BMS 告警字节 1
uint8 bms_alarm2                     # BMS 告警字节 2
uint8 bms_warning1                   # BMS 预警字节 1
uint8 bms_warning2                   # BMS 预警字节 2
uint8[7] remote                      # 遥控器原始 7 字节状态
