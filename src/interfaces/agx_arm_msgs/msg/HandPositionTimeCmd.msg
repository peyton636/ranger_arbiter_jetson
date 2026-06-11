# Revo2 灵巧手位置-时间混合控制命令
# 话题: /control/hand_position_time
# 适用条件: 末端执行器配置为 revo2
# 对应 SDK 的 position_time_ctrl()，同时指定各指目标位置与到达时间，实现平滑协调运动

# ---- 各手指目标位置 ----
# 范围 [0, 100]：0=完全张开，100=完全弯曲

# 拇指指尖目标位置
int8 thumb_tip_pos

# 拇指指根目标位置
int8 thumb_base_pos

# 食指目标位置
int8 index_finger_pos

# 中指目标位置
int8 middle_finger_pos

# 无名指目标位置
int8 ring_finger_pos

# 小指目标位置
int8 pinky_finger_pos

# ---- 各手指到达目标位置所需时间 ----
# 单位: 10 ms，范围 [0, 255]；例如 200 表示 2000 ms (2 秒)

# 拇指指尖到达时间
uint8 thumb_tip_time

# 拇指指根到达时间
uint8 thumb_base_time

# 食指到达时间
uint8 index_finger_time

# 中指到达时间
uint8 middle_finger_time

# 无名指到达时间
uint8 ring_finger_time

# 小指到达时间
uint8 pinky_finger_time
