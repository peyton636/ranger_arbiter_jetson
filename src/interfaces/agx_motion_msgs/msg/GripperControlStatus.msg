# 夹爪运行状态（发布到 /gripper/status，闭环反馈抓取结果）

std_msgs/Header header              # 消息时间戳
string request_id                   # 对应 GripperCmd 的 request_id

# ---------- 夹爪状态（state 填下面其中一个值）----------
uint8 STATE_IDLE = 0                # 空闲，无指令在执行
uint8 STATE_MOVING = 1              # 正在张开或闭合
uint8 STATE_GRASPED = 2             # 已夹住物体（力或电流达到阈值）
uint8 STATE_FAILED = 3              # 动作失败（超时、空抓等）
uint8 STATE_OPEN = 4                # 已完全张开
uint8 state

bool force_threshold_met            # 夹持力是否达到设定阈值（认为夹紧）
bool current_threshold_met          # 电机电流是否达到阈值（辅助判断有无物体）
float64 width                       # 当前夹爪开口宽度（米）
float64 force                       # 当前测得夹持力（牛）
string message                      # 状态说明或失败原因
