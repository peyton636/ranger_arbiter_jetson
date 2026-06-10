# STM32 -> Jetson, CAN ID 0x105, GPS fragment B (8B, magic 0xA4)
std_msgs/Header header

uint8 magic                 # fixed 0xA4
uint8 frag_idx              # fixed 1

int32 lat_e7                # latitude * 1e7
int16 heading_x100          # heading * 100, degrees
