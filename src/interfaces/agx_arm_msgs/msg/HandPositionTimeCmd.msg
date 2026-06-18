# Revo2 灵巧手位置-时间混合控制（position_time_ctrl）
# 同时下发目标位置与到达时间

# 各指目标位置 0~100
int8 thumb_tip_pos
int8 thumb_base_pos
int8 index_finger_pos
int8 middle_finger_pos
int8 ring_finger_pos
int8 pinky_finger_pos

# 各指到达时间，单位 10 ms，范围 0~255（200=2 s）
uint8 thumb_tip_time
uint8 thumb_base_time
uint8 index_finger_time
uint8 middle_finger_time
uint8 ring_finger_time
uint8 pinky_finger_time
