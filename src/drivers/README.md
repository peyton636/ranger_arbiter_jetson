## drivers_pkg
放 agv_base_driver_node、camera_driver_node、arm_driver_node、gripper_driver_node、T300_bridge_node、lidar_driver_node。
原则：只做设备接入、协议解析、基础状态发布，不写业务逻辑。
      （建议再细分：base_driver, camera_driver, arm_driver, gripper_driver, mcu_bridge, radar_driver）
        后续扩展方式：
        - 更换硬件时只换对应 adapter
        - 上层 topic 不变