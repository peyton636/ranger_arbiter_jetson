# Agx 机械臂状态反馈
uint8 ctrl_mode                     # 控制模式
uint8 arm_status                    # 机械臂状态
uint8 mode_feedback                 # 模式反馈
uint8 teach_status                  # 示教模式状态
uint8 motion_status                 # 运动状态：0=到位，1=未到位
uint8 trajectory_num                # 当前轨迹点序号
int64 err_status                    # 错误状态码
bool[] joint_angle_limit            # 各关节是否触达角度限位
bool[] communication_status_joint   # 各关节通信是否正常
