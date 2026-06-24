## drivers

| 目录/包名 | ROS 包名 | 节点/可执行文件 | 作用 |
|-----------|----------|-----------------|------|
| `agv_base_bringe/` | `agv_base_driver` | `agv_base_bringe_node` | 底盘业务：cmd_vel → V3Command，VehicleData/odom |
| `eth_gateway/` | `eth_gateway` | `eth_gateway`, `cmd_vel_gui` | MCU 以太网 UDP BLOB 网关 + 遥控 GUI |
| `gps_rs232_to_fix/` | `gps_rs232_to_fix` | `gps_to_fix` | GPS 帧 → `/fix` |
| `imu_adapter/` | `imu_adapter` | — | IMU 串口适配 |
| `rplidar/` | `rplidar_ros` | `rplidar_node` | 雷达 |

**以太网一键启动：**
```bash
ros2 launch agv_base_driver jetson_eth_bringup_gui.launch.py \
  bind_ip:=192.168.10.201 mcu_ip:=192.168.10.30
```

原则：只做设备接入、协议解析、基础状态发布，不写业务逻辑。