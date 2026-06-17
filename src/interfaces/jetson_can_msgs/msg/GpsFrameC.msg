# STM32 -> Jetson，CAN ID 0x106，GPS 帧 C（8 字节，魔数 0xA4）
std_msgs/Header header

uint8 magic                       # 固定 0xA4
uint8 frag_idx                    # 固定 2

int32 lon_e7                      # 经度 × 1e7（度）

int16 ALT_INVALID=32767           # 海拔无效值 0x7FFF
int16 alt_dm                      # 海拔 ×10，单位分米（实际海拔 m = alt_dm / 10.0）
