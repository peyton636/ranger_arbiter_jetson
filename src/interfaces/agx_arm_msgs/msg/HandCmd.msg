# Revo2 灵巧手控制命令
# 话题: /control/hand
# 适用条件: 末端执行器配置为 revo2
# 通过 mode 字段切换位置/速度/电流三种控制方式，各手指目标值含义随模式不同

# 控制模式，字符串枚举:
#   "position" — 位置控制，各指目标值范围 [0, 100]，0=完全张开，100=完全弯曲
#   "speed"    — 速度控制，各指目标值范围 [-100, 100]，负=张开方向，正=弯曲方向
#   "current"  — 电流控制，各指目标值范围 [-100, 100]，负=张开方向，正=弯曲方向
string mode

# 拇指指尖目标值，取值范围见上方 mode 说明
int8 thumb_tip

# 拇指指根目标值
int8 thumb_base

# 食指目标值
int8 index_finger

# 中指目标值
int8 middle_finger

# 无名指目标值
int8 ring_finger

# 小指目标值
int8 pinky_finger
