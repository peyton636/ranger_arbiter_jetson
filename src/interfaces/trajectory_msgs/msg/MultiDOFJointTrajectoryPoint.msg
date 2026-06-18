# 多自由度单点：位姿 + 速度 + 加速度 + 时间

# 各关节位姿（最多 6 DOF）
geometry_msgs/Transform[] transforms

# 各关节原点线/角速度
geometry_msgs/Twist[] velocities

# 各关节原点线/角加速度
geometry_msgs/Twist[] accelerations

# 从轨迹起点到该点的期望时间
builtin_interfaces/Duration time_from_start
