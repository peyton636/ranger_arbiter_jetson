# Orbbec 传感器外参（旋转 + 平移）
std_msgs/Header header
float64[9] rotation                 # 3×3 旋转矩阵（行优先）
float64[3] translation              # 平移向量 [x, y, z]
