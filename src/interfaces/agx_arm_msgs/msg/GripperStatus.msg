# Agx 夹爪状态反馈
std_msgs/Header header

float64 width                       # 当前开口宽度，单位 m
float64 force                       # 当前夹持力，单位 N

bool voltage_too_low                # 电压过低
bool motor_overheating              # 电机过热
bool driver_overcurrent             # 驱动过流
bool driver_overheating             # 驱动过热
bool sensor_status                  # 传感器状态
bool driver_error_status            # 驱动错误
bool driver_enable_status           # 驱动使能
bool homing_status                  # 回零/找原点状态
