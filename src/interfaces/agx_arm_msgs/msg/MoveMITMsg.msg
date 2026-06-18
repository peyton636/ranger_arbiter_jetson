# MIT 阻抗/力矩控制指令（多关节）
int32[] joint_index                 # 受控关节索引
float64[] p_des                     # 期望关节位置，单位 rad
float64[] v_des                     # 期望关节速度，单位 rad/s
float64[] kp                        # 位置增益
float64[] kd                        # 速度增益
float64[] torque                    # 期望关节力矩，单位 N·m
