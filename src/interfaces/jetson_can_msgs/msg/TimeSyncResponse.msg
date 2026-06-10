# STM32 -> Jetson, CAN ID 0x108 (8B)
std_msgs/Header header

uint32 system_tick_ms       # STM32 monotonic ms
uint32 utc_unix_sec         # Unix seconds since 1970-01-01 UTC, 0 if no GPS fix
