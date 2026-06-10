# STM32 -> Jetson, CAN ID 0x104, GPS fragment A (8B, magic 0xA4)
std_msgs/Header header

uint8 magic                 # fixed 0xA4
uint8 frag_idx              # fixed 0

# FLAGS bitfield: bit0 POS_VALID, bit1 VEL_VALID, bit2 HEADING_VALID,
# bit3 FIX_3D_LIKE, bit4 USEFULL
uint8 flags

uint8 num_sv

uint16 HDOP_INVALID=65535   # 0xFFFF when unknown
uint16 hdop_x100              # HDOP * 100

uint16 speed_cm_s             # ground speed, cm/s
