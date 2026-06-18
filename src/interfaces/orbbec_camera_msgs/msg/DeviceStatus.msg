# Orbbec 相机运行状态与帧率/延迟统计
std_msgs/Header header

# --- 彩色流 ---
float64 color_frame_rate_cur        # 彩色帧率当前值，Hz
float64 color_frame_rate_avg        # 彩色帧率平均值，Hz
float64 color_frame_rate_min        # 彩色帧率最小值，Hz
float64 color_frame_rate_max        # 彩色帧率最大值，Hz

float64 color_delay_ms_cur          # 彩色帧延迟当前值，ms
float64 color_delay_ms_avg          # 彩色帧延迟平均值，ms
float64 color_delay_ms_min          # 彩色帧延迟最小值，ms
float64 color_delay_ms_max          # 彩色帧延迟最大值，ms

# --- 深度流 ---
float64 depth_frame_rate_cur        # 深度帧率当前值，Hz
float64 depth_frame_rate_avg        # 深度帧率平均值，Hz
float64 depth_frame_rate_min        # 深度帧率最小值，Hz
float64 depth_frame_rate_max        # 深度帧率最大值，Hz

float64 depth_delay_ms_cur          # 深度帧延迟当前值，ms
float64 depth_delay_ms_avg          # 深度帧延迟平均值，ms
float64 depth_delay_ms_min          # 深度帧延迟最小值，ms
float64 depth_delay_ms_max          # 深度帧延迟最大值，ms

# --- 设备 ---
bool device_online                  # 设备是否在线
string connection_type              # 连接类型，如 "USB2.0"、"USB3.0"、"GigE"

# --- 标定 ---
bool customer_calibration_ready     # 用户标定是否就绪
bool calibration_from_factory       # 是否使用出厂标定
bool calibration_from_launch_param  # 是否来自 launch 参数标定
