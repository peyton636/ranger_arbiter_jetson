# BLOB MSG 0x08 agv_energy_t
std_msgs/Header header
uint32 mcu_timestamp_ms
uint8 blob_seq
int32[4] odom
uint8 bms_soc
uint8 bms_soh
uint16 bms_voltage_0p1v
int16 bms_current
int16 bms_temperature
uint8 bms_alarm1
uint8 bms_alarm2
uint8 bms_warning1
uint8 bms_warning2
uint8[7] remote
