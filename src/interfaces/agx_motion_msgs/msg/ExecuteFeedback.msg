# 规划/执行过程反馈（发布到 /motion/execute_feedback，实时汇报进度）

std_msgs/Header header              # 消息时间戳
string request_id                   # 对应 PlanRequest 的 request_id

# ---------- 当前状态（status 填下面其中一个值）----------
uint8 STATUS_IDLE = 0               # 空闲，无任务
uint8 STATUS_PLANNING = 1           # 正在调用 MoveIt 规划
uint8 STATUS_PLANNED = 2            # 规划完成，轨迹已发布
uint8 STATUS_EXECUTING = 3          # 等待或正在执行轨迹（arm_controller 执行中）
uint8 STATUS_SUCCEEDED = 4          # 全部成功（规划 + 执行，若 execute=true）
uint8 STATUS_FAILED = 5             # 失败（规划失败或执行失败）
uint8 STATUS_ABORTED = 6            # 被中止
uint8 status

float64 progress                    # 进度 0.0~1.0（如规划中 0、规划完 0.5、执行完 1.0）
string message                      # 人类可读说明，如 "planning"、"execution succeeded"
