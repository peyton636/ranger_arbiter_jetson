## 从2D转3D后的目标

std_msgs/Header header          # frame_id=目标所在坐标系, 推荐 base_link
int32 track_id
string class_name
int32 class_id
float32 confidence_2d
float32 confidence_3d           # 几何解算可信度 [0,1]

string source_frame             # 例如 camera_color_optical_frame
string target_frame             # 例如 base_link
bool depth_valid
bool tf_valid

geometry_msgs/Point position    # 目标点(米)

# 目标主方向(可选)，单位向量
geometry_msgs/Vector3 axis_x
geometry_msgs/Vector3 axis_y
geometry_msgs/Vector3 axis_z

# 3x3 协方差, 行主序
float64[9] covariance