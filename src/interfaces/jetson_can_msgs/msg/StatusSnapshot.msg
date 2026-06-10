# STM32 -> Jetson, CAN ID 0x10B, response to 0x10A query (8B)
std_msgs/Header header

uint8 safety_state          # same as V3Status byte3
uint8 link_state            # same as V3Status byte4

int16 fb_v_mm_s
uint16 NEAREST_INVALID=65535
uint16 nearest_mm           # nearest obstacle mm, 0xFFFF invalid

uint8 limit_factor          # 0~100 percent

# FLAGS bitfield: bit0 GPS_FIX, bit1 CHASSIS_CAN_ACTIVE, bit2 DIST_VALID
uint8 flags
