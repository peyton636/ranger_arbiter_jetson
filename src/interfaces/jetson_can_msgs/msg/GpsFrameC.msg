# STM32 -> Jetson, CAN ID 0x106, GPS fragment C (8B, magic 0xA4)
std_msgs/Header header

uint8 magic                 # fixed 0xA4
uint8 frag_idx              # fixed 2

int32 lon_e7                # longitude * 1e7

int16 ALT_INVALID=32767     # 0x7FFF when invalid
int16 alt_dm                # altitude * 10, decimeters
