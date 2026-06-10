# STM32 -> Jetson, CAN ID 0x103, V3 frame type 0x03 (24B reassembled)
std_msgs/Header header

uint8 seq

int16 wheel_rf
int16 wheel_rr
int16 wheel_lr
int16 wheel_lf

int16 steer_rf_millirad
int16 steer_rr_millirad
int16 steer_lr_millirad
int16 steer_lf_millirad

int8 motor_temp_max_c
uint8 driver_state_or       # 8-way driver status bitwise OR
