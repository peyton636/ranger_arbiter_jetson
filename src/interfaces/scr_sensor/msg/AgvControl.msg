# 统一 AGV 业务控制（Topic: /agv_control）
# 上层导航 / GUI / 任务 → agv_base_bringe → /jetson_{link}/command (V3Command，内部)
std_msgs/Header header

# --- 控制模式 MODE_REQ（与 V3Command 一致）---
uint8 MODE_STANDBY=0
uint8 MODE_CAN=1
uint8 MODE_REMOTE=2
uint8 mode_req

# --- 直接运动指令（use_twist_input=false 时生效）---
int16 v_mm_s                      # 线速度 mm/s
int16 omega_millirad_s            # 自旋角速度 0.001 rad/s
int16 steer_millirad              # 转角 0.001 rad

uint8 MOTION_ACKERMANN=0
uint8 MOTION_SIDEWAYS=1
uint8 MOTION_SPIN=2
uint8 MOTION_PARK=3
uint8 motion_model

uint8 light_enable
uint8 light_mode
uint8 clear_error

# --- 可选：Twist 语义输入（GUI / Nav2 兼容）---
bool use_twist_input
float64 linear_x                  # m/s
float64 linear_y                  # m/s
float64 angular_z                 # rad/s
