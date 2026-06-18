# Orbbec RGBD 同步帧（彩色 + 深度 + 相机内参）
std_msgs/Header header
sensor_msgs/CameraInfo rgb_camera_info    # RGB 相机内参
sensor_msgs/CameraInfo depth_camera_info  # 深度相机内参
sensor_msgs/Image rgb                     # RGB 图像
sensor_msgs/Image depth                   # 深度图像
