# 运动规划请求消息
# 话题: /motion/plan_request

std_msgs/Header header  # 消息头，包含时间戳和坐标系信息

# 规划类型枚举
uint8 PLAN_TYPE_JOINT = 0        # 关节空间规划
uint8 PLAN_TYPE_NAMED_TARGET = 1 # 预定义命名目标规划
uint8 PLAN_TYPE_POSE = 2         # 位姿目标规划
uint8 PLAN_TYPE_CARTESIAN = 3    # 笛卡尔方向规划
uint8 plan_type                  # 当前请求使用的规划类型

string request_id                # 请求 ID，用于区分不同规划请求
string group_name                # 规划组名称，例如机械臂名称
string planner_id                # 指定规划器 ID

# PLAN_TYPE_JOINT：关节目标值
float64[] joint_goal             # 各关节的目标角度/位置

# PLAN_TYPE_NAMED_TARGET：命名目标
string named_target              # 预定义目标名称

# PLAN_TYPE_POSE：末端位姿目标（也可由 /grasp/selected_pose 覆盖）
geometry_msgs/PoseStamped pose_goal  # 目标位姿

# PLAN_TYPE_CARTESIAN：沿指定方向进行笛卡尔规划
geometry_msgs/Vector3Stamped cartesian_direction  # 相对当前位姿的移动方向
float64 cartesian_min_dist                        # 最小移动距离
float64 cartesian_max_dist                        # 最大移动距离
float64 cartesian_step_size                       # 采样步长

bool execute                      # 是否在规划成功后直接执行
float64 max_velocity_scaling      # 最大速度缩放比例
float64 max_acceleration_scaling  # 最大加速度缩放比例
