# STM32 -> Jetson，CAN ID 0x105，GPS 帧 B（8 字节，魔数 0xA4）
std_msgs/Header header

uint8 magic                       # 固定 0xA4
uint8 frag_idx                    # 固定 1

int32 lat_e7                      # 纬度 × 1e7（度）
int16 heading_x100                # 航向角 ×100（度）
