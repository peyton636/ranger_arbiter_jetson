## MoveIt/执行层的抓取候选

std_msgs/Header header          # frame_id=base_link
int32 track_id
string class_name
float32 score                   # 综合评分 [0,1]

# 三段式姿态
geometry_msgs/PoseStamped pre_grasp_pose
geometry_msgs/PoseStamped grasp_pose
geometry_msgs/PoseStamped retreat_pose

# 末端动作建议
float32 gripper_width           # m
float32 gripper_force           # N

# 运动学与执行约束
float32 approach_distance       # m
float32 retreat_distance        # m
float32 max_cartesian_speed     # m/s
float32 max_joint_speed_ratio   # 0~1