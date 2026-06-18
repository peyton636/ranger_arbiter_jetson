# Jetson -> STM32，CAN ID 0x101，V3 下行控制帧（type=0x01，24 字节组帧后）
# Topic（发布）：/jetson_eth/command、/jetson_rs232/command、/jetson_can/command
std_msgs/Header header

uint8 seq                         # Jetson 发送序号

# 控制模式请求 MODE_REQ
uint8 MODE_STANDBY=0              # 待机
uint8 MODE_CAN=1                  # CAN 指令控制
uint8 MODE_REMOTE=2               # 遥控请求
uint8 mode_req

int16 v_mm_s                      # 线速度指令，单位 mm/s
int16 omega_millirad_s            # 自旋角速度指令，单位 0.001 rad/s
int16 steer_millirad              # 转角指令，单位 0.001 rad

# 运动模式 MOTION_MODEL
uint8 MOTION_ACKERMANN=0          # 前后阿克曼
uint8 MOTION_SIDEWAYS=1           # 斜移
uint8 MOTION_SPIN=2               # 自旋
uint8 MOTION_PARK=3               # 驻车
uint8 motion_model

uint8 light_enable                # 灯光使能
uint8 light_mode                  # 灯光模式
uint8 clear_error                 # 清错请求
