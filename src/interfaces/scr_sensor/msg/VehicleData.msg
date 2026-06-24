# AGV 车体量测（Topic: /Vehicle/VehicleData）
std_msgs/Header header

# --- 运动反馈 ---
int16 linear_velocity_mm_s
int16 angular_velocity_millirad_s
int16 steer_millirad
int16 steer_rf_millirad
int16 steer_rr_millirad
int16 steer_lr_millirad
int16 steer_lf_millirad

# --- 四向超声 ---
uint16 SONAR_INVALID=65535
uint16 sonar_front_mm
uint16 sonar_back_mm
uint16 sonar_left_mm
uint16 sonar_right_mm
uint8 sonar_source                  # 0=MCU状态 1=传感器BLOB
bool sonar_time_sync_valid

# --- 电池 / BMS ---
uint16 battery_voltage_0p1v
uint8 battery_soc_percent
uint8 bms_soh_percent
int16 bms_current
int16 bms_temperature
uint8 bms_alarm1
uint8 bms_alarm2
uint8 bms_warning1
uint8 bms_warning2
int32[4] odom_pulse                  # 四轮里程计脉冲 [RF,RR,LR,LF]

# --- 轮速 / 电机 ---
int16 wheel_rf
int16 wheel_rr
int16 wheel_lr
int16 wheel_lf
int8 motor_temp_max_c
uint8 driver_state_or
AgvMotorCompact[8] motors
int32[8] motor_position

uint8 v3_status_seq
uint8 v3_ext_status_seq
bool data_valid
