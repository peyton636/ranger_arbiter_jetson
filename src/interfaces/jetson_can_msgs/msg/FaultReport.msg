# STM32 -> Jetson，CAN ID 0x109，故障上报（8 字节）
std_msgs/Header header

uint8 fault_code_1                # 故障码 1（见协议统一错误码表）
uint8 fault_code_2                # 故障码 2，无则为 0
uint8 fault_code_3                # 故障码 3
uint8 fault_code_4                # 故障码 4

uint16 fault_ts_ms                # 故障发生时 SYSTEM_TICK_MS 低 16 位

uint8 chassis_sys                 # 底盘 0x211 帧 system_status 原样
uint8 chassis_fault_b0            # 底盘 0x211 帧 FAULT_INFO 最低字节
