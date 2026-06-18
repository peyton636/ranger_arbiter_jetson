# 运动规划请求 — 话题: /motion/plan_request

std_msgs/Header header

# 规划类型
uint8 PLAN_TYPE_JOINT = 0
uint8 PLAN_TYPE_NAMED_TARGET = 1
uint8 PLAN_TYPE_POSE = 2
uint8 PLAN_TYPE_CARTESIAN = 3
uint8 plan_type

string request_id
string group_name
string planner_id

# PLAN_TYPE_JOINT
float64[] joint_goal

# PLAN_TYPE_NAMED_TARGET
string named_target

# PLAN_TYPE_POSE（也可由 /grasp/selected_pose 覆盖）
geometry_msgs/PoseStamped pose_goal

# PLAN_TYPE_CARTESIAN：相对当前位姿沿该方向移动
geometry_msgs/Vector3Stamped cartesian_direction
float64 cartesian_min_dist
float64 cartesian_max_dist
float64 cartesian_step_size

bool execute
float64 max_velocity_scaling
float64 max_acceleration_scaling
