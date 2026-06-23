# 规划得到的运动轨迹（发布到 /motion/trajectory，由 arm_controller_node 订阅执行）

std_msgs/Header header              # 消息时间戳
string request_id                   # 对应 PlanRequest 的 request_id，用于串联规划与执行
moveit_msgs/RobotTrajectory trajectory  # MoveIt 规划结果：含关节名、各时刻关节位置/速度
