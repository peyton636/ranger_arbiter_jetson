# 夹爪控制指令 — 话题: /task/gripper_cmd

std_msgs/Header header
string request_id

uint8 CMD_OPEN = 0
uint8 CMD_CLOSE = 1
uint8 CMD_STOP = 2
uint8 command

float64 target_width
float64 max_force
bool wait_grasp
