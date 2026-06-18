# Revo2 灵巧手状态反馈
std_msgs/Header header

uint8 left_or_right                 # 1=左手，2=右手

# 各指位置 0~100
uint8 thumb_tip_pos
uint8 thumb_base_pos
uint8 index_finger_pos
uint8 middle_finger_pos
uint8 ring_finger_pos
uint8 pinky_finger_pos

# 各指电机状态：0=空闲，1=运行，2=堵转
uint8 thumb_tip_status
uint8 thumb_base_status
uint8 index_finger_status
uint8 middle_finger_status
uint8 ring_finger_status
uint8 pinky_finger_status
