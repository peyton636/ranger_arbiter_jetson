# MCU -> Jetson，BLOB MSG 0x03，mcu_status_t，42 B PAYLOAD
# Topic：/jetson_eth/blob/mcu_status、/jetson_rs232/blob/mcu_status
std_msgs/Header header

uint32 mcu_timestamp_ms           # MCU 帧时刻，单位 ms
uint8 blob_seq                     # BLOB SEQ
uint8 seq                          # MCU 状态帧序号
# 安全性：0x01 正常 0x02 限速 0x03 降级 0x04 紧急
uint8 safety_state
# 链路位：bit0=Jetson 心跳丢失 bit1=底盘故障 bit2=蜂鸣器
uint8 link_state
uint8 limit_factor                 # 限速比例 0~100（%）
int16 arb_v_mm_s                     # 仲裁后线速度，单位 mm/s
int16 arb_omega_millirad_s           # 仲裁后角速度，单位 0.001 rad/s
int16 arb_steer_millirad             # 仲裁后转角，单位 0.001 rad
uint16[4] sonar_mm                   # 四向超声 [前, 后, 左, 右]，单位 mm；无效 0xFFFF
uint32[4] sonar_stamp_ms             # 四路超声 stable_mm 最后变化时刻（MCU tick）
uint32[4] sonar_jetson_mono_ms      # sonar_stamp 换算到 Jetson monotonic ms；无效为 0
bool time_sync_valid                 # offset 有效时 sonar_jetson_mono_ms 可用
uint16 nearest_mm                    # 最近障碍距离，单位 mm
uint8 jetson_seq                     # 最近收到的 Jetson 下行 0x01 SEQ（回环校验）
