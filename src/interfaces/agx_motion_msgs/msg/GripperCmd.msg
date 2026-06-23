# 夹爪控制指令（发布到 /task/gripper_cmd，由 gripper_controller_node 执行）

std_msgs/Header header              # 消息时间戳
string request_id                   # 请求 ID，用于匹配 GripperControlStatus 反馈

# ---------- 动作类型（command 填下面其中一个值）----------
uint8 CMD_OPEN = 0                  # 张开夹爪
uint8 CMD_CLOSE = 1                 # 闭合夹爪（可配合力/宽度限制做抓取）
uint8 CMD_STOP = 2                  # 立即停止当前动作
uint8 command

float64 target_width                # 目标开口宽度（米），张开/闭合时的目标开度
float64 max_force                   # 最大夹持力（牛），闭合时超过则停止施力
bool wait_grasp                     # true：闭合后等待检测到抓取成功再返回状态
