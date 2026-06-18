# 关节空间轨迹（ROS trajectory_msgs 兼容）
# header 指定坐标系与轨迹时间基准
std_msgs/Header header

# 各轨迹点中的关节名，顺序与 JointTrajectoryPoint 中数组一致
string[] joint_names

# 轨迹点序列：各时刻的位置/速度/加速度/力矩
JointTrajectoryPoint[] points
