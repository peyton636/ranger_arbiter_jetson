# STM32 -> Jetson, CAN ID 0x109 (8B)
std_msgs/Header header

uint8 fault_code_1
uint8 fault_code_2
uint8 fault_code_3
uint8 fault_code_4

uint16 fault_ts_ms          # SYSTEM_TICK_MS low 16 bits at fault time

uint8 chassis_sys           # chassis 0x211 system_status
uint8 chassis_fault_b0      # chassis 0x211 FAULT_INFO lowest byte
