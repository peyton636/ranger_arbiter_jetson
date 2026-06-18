# BLOB 单路电机紧凑体 motor_compact_t（10 B），用于 agv_motor04/58_t
int16 speed_rpm                     # 转速，单位 rpm
int16 current                       # 电流
uint16 voltage                      # 电压
int8 temperature                    # 温度，单位 ℃
uint8 driver_status                 # 驱动器状态字
uint16 position_lo                  # 位置低 16 位（与 0x0B 全脉冲配合）
