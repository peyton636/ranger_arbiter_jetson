# AgxGripper 夹爪状态反馈
# 话题: /feedback/gripper_status
# 适用条件: 末端执行器配置为 agx_gripper
# 周期性反馈夹爪开口宽度、夹持力及驱动器健康状态


std_msgs/Header header      # 消息头，含时间戳与坐标系 frame_id

# ---- 夹爪运动状态 ----

float64 width       # 当前夹爪开口宽度，单位: m

float64 force       # 当前夹持力，单位: N

# ---- 驱动器状态标志 ----
# 以下 bool 字段中，故障类标志 true=异常/false=正常；
# 使能与回零标志语义见各字段注释

bool voltage_too_low        # 供电电压过低

bool motor_overheating      # 电机过热

bool driver_overcurrent     # 驱动器过流

bool driver_overheating     # 驱动器过热

bool sensor_status          # 传感器异常

bool driver_error_status    # 驱动器存在错误

bool driver_enable_status   # 驱动器使能状态: true=已使能, false=未使能

bool homing_status          # 回零/归零状态: true=已完成回零, false=未完成
