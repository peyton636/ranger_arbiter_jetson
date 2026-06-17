# STM32 -> Jetson，CAN ID 0x10B，状态查询响应（8 字节，回复 0x10A 请求）
std_msgs/Header header

uint8 safety_state                # 同 V3Status 的 safety_state
uint8 link_state                  # 同 V3Status 的 link_state

int16 fb_v_mm_s                   # 当前线速度，单位 mm/s

uint16 NEAREST_INVALID=65535      # 最近障碍距离无效值 0xFFFF
uint16 nearest_mm                 # 最近障碍距离，单位 mm

uint8 limit_factor                # 限速比例 0~100（%）

# 快照标志位 FLAGS：bit0=GPS定位 bit1=底盘CAN模式 bit2=至少一路测距有效
uint8 flags
