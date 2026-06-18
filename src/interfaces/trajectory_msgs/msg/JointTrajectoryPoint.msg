# 单个关节轨迹点
# 字段顺序与 JointTrajectory.joint_names 一致

# 关节位置（相对零位）：旋转关节 rad，移动关节 m
float64[] positions

# 关节速度：rad/s 或 m/s
float64[] velocities

# 关节加速度：rad/s² 或 m/s²
float64[] accelerations

# 关节力矩/力：旋转关节 N·m，移动关节 N
float64[] effort

# 从轨迹起点到该点的期望时间
builtin_interfaces/Duration time_from_start
