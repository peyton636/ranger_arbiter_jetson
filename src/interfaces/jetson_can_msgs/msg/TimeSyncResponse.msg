# STM32 -> Jetson，CAN ID 0x108 / 0xA5 服务帧，时间同步响应（8 字节）
# Topic：/jetson_eth/time_sync、/jetson_rs232/time_sync、/jetson_can/time_sync
std_msgs/Header header

uint32 system_tick_ms             # STM32 单调毫秒计数
uint32 utc_unix_sec               # Unix 时间戳（1970-01-01 UTC 起秒数），无 GPS 时为 0
uint8 cmd_echo                    # 回显命令：0x01=QUERY 0x02=START 0x03=PING
uint8 seq_echo                    # PING seq 或 START session_id
float32 rtt_ms                    # 往返时延（PING/START）
float32 offset_ms                 # mcu_tick ≈ jetson_mono + offset（EMA）
float32 proc_ms                   # MCU 处理耗时
bool gps_utc_valid                # byte6 bit0
