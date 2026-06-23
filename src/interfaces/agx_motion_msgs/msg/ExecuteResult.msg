# 轨迹执行最终结果（发布到 /motion/execute_result，由 arm_controller_node 回传）

std_msgs/Header header              # 消息时间戳
string request_id                   # 对应 PlanRequest / MotionTrajectory 的 request_id
bool success                        # true：机械臂已按轨迹执行到位；false：超时或执行失败
int32 error_code                    # 错误码，0 表示无错误（具体含义见 arm_controller 实现）
string message                      # 失败原因或补充说明
