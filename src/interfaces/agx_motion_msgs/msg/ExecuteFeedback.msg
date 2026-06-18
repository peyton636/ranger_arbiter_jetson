# 规划/执行过程反馈 — 话题: /motion/execute_feedback

std_msgs/Header header
string request_id

uint8 STATUS_IDLE = 0
uint8 STATUS_PLANNING = 1
uint8 STATUS_PLANNED = 2
uint8 STATUS_EXECUTING = 3
uint8 STATUS_SUCCEEDED = 4
uint8 STATUS_FAILED = 5
uint8 STATUS_ABORTED = 6
uint8 status

float64 progress
string message
