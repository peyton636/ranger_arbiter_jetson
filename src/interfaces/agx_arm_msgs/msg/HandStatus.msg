# Revo2 灵巧手状态反馈
# 话题: /feedback/hand_status
# 适用条件: 末端执行器配置为 revo2
# 周期性反馈各手指当前位置与电机运行状态

# 消息头，含时间戳与坐标系 frame_id
std_msgs/Header header

# 手部类型标识: 1=左手, 2=右手
uint8 left_or_right

# ---- 各手指当前位置 ----
# 范围 [0, 100]：0=完全张开，100=完全弯曲

# 拇指指尖当前位置
uint8 thumb_tip_pos

# 拇指指根当前位置
uint8 thumb_base_pos

# 食指当前位置
uint8 index_finger_pos

# 中指当前位置
uint8 middle_finger_pos

# 无名指当前位置
uint8 ring_finger_pos

# 小指当前位置
uint8 pinky_finger_pos

# ---- 各手指电机运行状态 ----
# 0=空闲(idle), 1=运行中(running), 2=堵转/卡死(blocked)

# 拇指指尖电机状态
uint8 thumb_tip_status

# 拇指指根电机状态
uint8 thumb_base_status

# 食指电机状态
uint8 index_finger_status

# 中指电机状态
uint8 middle_finger_status

# 无名指电机状态
uint8 ring_finger_status

# 小指电机状态
uint8 pinky_finger_status
