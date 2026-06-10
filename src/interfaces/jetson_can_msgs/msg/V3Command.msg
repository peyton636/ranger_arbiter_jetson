# Jetson -> STM32, CAN ID 0x101, V3 frame type 0x01 (24B reassembled)
std_msgs/Header header

uint8 seq

# MODE_REQ
uint8 MODE_STANDBY=0
uint8 MODE_CAN=1
uint8 MODE_REMOTE=2
uint8 mode_req

int16 v_mm_s
int16 omega_millirad_s
int16 steer_millirad

# MOTION_MODEL
uint8 MOTION_ACKERMANN=0
uint8 MOTION_SIDEWAYS=1
uint8 MOTION_SPIN=2
uint8 MOTION_PARK=3
uint8 motion_model

uint8 light_enable
uint8 light_mode
uint8 clear_error
