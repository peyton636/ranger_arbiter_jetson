# Orbbec IMU 噪声与标定参数
std_msgs/Header header
float64 noise_density               # 噪声密度
float64 random_walk                 # 随机游走系数
float64 reference_temperature       # 参考温度
float64[3] bias                     # 零偏 [x, y, z]
float64[3] gravity                  # 重力向量
float64[9] scale_misalignment      # 刻度/不对准矩阵 3×3
float64[9] temperature_slope         # 温度斜率 3×3
