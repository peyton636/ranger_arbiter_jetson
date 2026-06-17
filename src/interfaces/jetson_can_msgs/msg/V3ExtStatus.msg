# STM32 -> Jetson，CAN ID 0x103，V3 上行扩展帧（type=0x03，24 字节组帧后）
std_msgs/Header header

uint8 seq                         # STM32 发送序号

int16 wheel_rf                    # 右前轮速（固件原始单位）
int16 wheel_rr                    # 右后轮速
int16 wheel_lr                    # 左后轮速
int16 wheel_lf                    # 左前轮速

int16 steer_rf_millirad           # 右前转角，单位 0.001 rad
int16 steer_rr_millirad           # 右后转角
int16 steer_lr_millirad           # 左后转角
int16 steer_lf_millirad           # 左前转角

int8 motor_temp_max_c             # 8 路电机最高温度，单位 ℃
uint8 driver_state_or             # 8 路驱动器状态按位 OR
