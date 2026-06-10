# drivers

硬件与底层通信驱动。

| 包 | 说明 |
|----|------|
| `ds_jetson_bridge` | Jetson ↔ STM32B V3 串口桥（`/cmd_vel` ↔ 24B） |
| `ds_can_monitor` | CAN 监视（SocketCAN 调试） |
| `ds_serial_monitor` | 串口监视 / pl2303 驱动辅助 |
| `ds_gps_driver` | GPS 串口驱动（`/fix`） |
| `ds_imu_driver` | WIT IMU 串口驱动（`/imu/data`） |
| `ranger_ros2` | AgileX Ranger ROS2（本地 clone，见 `.gitignore`） |
| `ugv_sdk` | UGV SDK（本地 clone，见 `.gitignore`） |

```bash
cd ~/catkin_ws/cangyirobot/src/drivers
git clone https://github.com/agilexrobotics/ranger_ros.git ranger_ros2
git clone --recursive https://github.com/agilexrobotics/ugv_sdk.git
```
