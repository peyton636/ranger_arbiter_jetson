# AGV 功能 / 安全 / 模式（Topic: /Function/FeatureStatusInfo）
std_msgs/Header header

uint8 SAFETY_NORMAL=1
uint8 SAFETY_SPEED_LIMIT=2
uint8 SAFETY_DEGRADED=3
uint8 SAFETY_EMERGENCY=4
uint8 safety_state

uint8 link_state                    # bit0=Jetson心跳 bit1=底盘故障 bit2=蜂鸣器
uint8 limit_factor

uint8 MOTION_ACKERMANN=0
uint8 MOTION_SIDEWAYS=1
uint8 MOTION_SPIN=2
uint8 MOTION_PARK=3
uint8 motion_model

uint32 fault_code
uint8 system_info
uint8 light_enable_feedback
uint8 light_mode_feedback

int16 arb_v_mm_s
int16 arb_omega_millirad_s
int16 arb_steer_millirad
uint16 nearest_obstacle_mm
uint8 jetson_seq
bool jetson_link_up

bool time_sync_valid
float64 time_sync_offset_ms

uint8 v3_status_seq
bool data_valid
