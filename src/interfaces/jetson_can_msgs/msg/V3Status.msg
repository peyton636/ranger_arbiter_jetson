# STM32 -> Jetson, CAN ID 0x102, V3 frame type 0x02 (24B reassembled)
std_msgs/Header header

uint8 seq

# SAFETY_STATE
uint8 SAFETY_NORMAL=1
uint8 SAFETY_SPEED_LIMIT=2
uint8 SAFETY_DEGRADED=3
uint8 SAFETY_EMERGENCY=4
uint8 safety_state

# LINK_STATE bitfield: bit0 JETSON_HB_LOST, bit1 CHASSIS_FAULT, bit2 BEEP_ACTIVE
uint8 link_state

uint8 limit_factor          # 0~100 percent

int16 fb_v_mm_s
int16 fb_omega_millirad_s   # 0.001 rad/s per count
int16 fb_steer_millirad     # 0.001 rad per count

uint16 SONAR_INVALID=65535  # 0xFFFF
uint16 sonar_front_mm
uint16 sonar_back_mm
uint16 sonar_left_mm
uint16 sonar_right_mm

uint16 battery_voltage_0p1v   # 0.1 V per count
uint8 battery_soc             # percent
