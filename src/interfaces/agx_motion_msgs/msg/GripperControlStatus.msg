# 夹爪闭环状态 — 话题: /gripper/status

std_msgs/Header header
string request_id

uint8 STATE_IDLE = 0
uint8 STATE_MOVING = 1
uint8 STATE_GRASPED = 2
uint8 STATE_FAILED = 3
uint8 STATE_OPEN = 4
uint8 state

bool force_threshold_met
bool current_threshold_met
float64 width
float64 force
string message
