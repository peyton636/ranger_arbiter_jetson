# STM32 -> Jetson，CAN ID 0x104，GPS 帧 A（8 字节，魔数 0xA4）
# Topic：/jetson_eth/gps/a、/jetson_rs232/gps/a
std_msgs/Header header

uint8 magic                       # 固定 0xA4
uint8 frag_idx                    # 固定 0

# 定位状态位 FLAGS：bit0=位置有效 bit1=速度有效 bit2=航向有效
# bit3=类3D定位 bit4=NMEA有效标志A
uint8 flags

uint8 num_sv                        # 卫星数

uint16 HDOP_INVALID=65535         # HDOP 未知时的无效值 0xFFFF
uint16 hdop_x100                    # 水平精度因子 ×100

uint16 speed_cm_s                   # 对地速度，单位 cm/s
