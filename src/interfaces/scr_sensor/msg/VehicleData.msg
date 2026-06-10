# AGV chassis aggregated state published by agv_base_driver_node.
# Source: jetson_can_msgs/V3Status (0x102) + V3ExtStatus (0x103), fused on Orin.
std_msgs/Header header

# --- Motion feedback (V3Status 0x102) ---
int16 linear_velocity_mm_s
int16 angular_velocity_millirad_s   # 0.001 rad/s per count
int16 steer_millirad                # 0.001 rad per count

# --- Safety / arbiter (V3Status 0x102) ---
uint8 SAFETY_NORMAL=1
uint8 SAFETY_SPEED_LIMIT=2
uint8 SAFETY_DEGRADED=3
uint8 SAFETY_EMERGENCY=4
uint8 safety_state

uint8 limit_factor                  # 0~100 percent

# link_state bit0 JETSON_HB_LOST, bit1 CHASSIS_FAULT, bit2 BEEP_ACTIVE
uint8 link_state

# --- Ultrasonic ranging mm (V3Status 0x102) ---
uint16 SONAR_INVALID=65535
uint16 sonar_front_mm
uint16 sonar_back_mm
uint16 sonar_left_mm
uint16 sonar_right_mm

# --- Power (V3Status 0x102) ---
uint16 battery_voltage_0p1v         # 0.1 V per count
uint8 battery_soc_percent

# --- Extended chassis (V3ExtStatus 0x103, latest) ---
int16 wheel_rf
int16 wheel_rr
int16 wheel_lr
int16 wheel_lf
int8 motor_temp_max_c
uint8 driver_state_or

# --- Freshness ---
uint8 v3_status_seq
uint8 v3_ext_status_seq
bool data_valid                     # true if status received within timeout
