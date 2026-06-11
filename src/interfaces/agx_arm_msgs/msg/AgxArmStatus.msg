# 机械臂整体状态反馈消息
# 话题: /feedback/arm_status
# 由 agx_arm 驱动节点周期性发布，反映当前控制模式、运行状态、示教状态及关节健康信息


uint8 ctrl_mode     # 控制模式 (ctrl_mode)
                    # 0=待机, 1=CAN指令控制, 2=示教模式, 3=以太网控制, 4=WiFi控制,
                    # 5=遥控模式, 6=联动示教输入, 7=离线轨迹模式, 8=TCP控制


uint8 arm_status    # 机械臂状态 (arm_status)
                    # 0=正常, 1=急停, 2=无解, 3=奇异点, 4=目标角度超限, 5=关节通信异常,
                    # 6=关节刹车未释放, 7=发生碰撞, 8=示教拖动超速, 9=关节状态异常,
                    # 10=其他异常, 11=示教记录中, 12=示教执行中, 13=示教暂停,
                    # 14=主控NTC过温, 15=释放电阻NTC过温


uint8 mode_feedback # 当前运动模式反馈 (mode_feedback)
                    # 0=MOVE P(点到点), 1=MOVE J(关节), 2=MOVE L(直线),
                    # 3=MOVE C(圆弧), 4=MOVE MIT(力矩), 5=MOVE CPV


uint8 teach_status  # 示教状态 (teach_status)
                    # 0=关闭, 1=开始示教记录, 2=结束示教记录, 3=执行示教轨迹,
                    # 4=暂停执行, 5=继续执行, 6=终止执行, 7=移动至轨迹起点


uint8 motion_status # 运动到位状态 (motion_status)
                    # 0=已到达目标位置, 1=未到达目标位置


uint8 trajectory_num    # 当前轨迹点序号 (trajectory_num)
                        # 范围 0~255，仅在离线轨迹模式 (ctrl_mode=7) 下有效


int64 err_status        # 错误状态码 (err_status)
                        # 控制器上报的故障/告警位域编码，具体含义请参考对应机型 SDK 文档


bool[] joint_angle_limit    # 各关节角度超限标志 (joint_angle_limit)
                            # 数组下标 0~6 对应关节 1~7；true=角度超限(异常), false=正常


bool[] communication_status_joint   # 各关节通信状态 (communication_status_joint)
                                    # 数组下标 0~6 对应关节 1~7；true=通信异常, false=正常
