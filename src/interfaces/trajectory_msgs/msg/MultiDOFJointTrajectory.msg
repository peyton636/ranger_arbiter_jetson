# 多自由度关节轨迹（每点为 Transform + Twist）
std_msgs/Header header

# 关节名；与各 MultiDOFJointTrajectoryPoint 数组长度、顺序一致
string[] joint_names

MultiDOFJointTrajectoryPoint[] points
