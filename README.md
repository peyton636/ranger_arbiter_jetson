# ranger_arbiter_jetson

Jetson ↔ STM32B ↔ Ranger Mini 联调工程（ROS2 Humble + Protocol V3）。

## 文档

详见 [docs/README.md](docs/README.md)（编译、串口接线、RS232 数据流、联调步骤、**GPS 模块联调**）。

## 主要包

| 包 | 说明 |
|----|------|
| `src/ds_jetson_bridge` | Jetson V3 串口桥（`/cmd_vel` ↔ 24B UART） |
| `src/ds_serial_monitor` | 串口监视 / pl2303 驱动辅助 |
| `src/ds_gps_driver` | ROS2 GPS 串口驱动（`/fix`） |
| `src/ds_imu_driver` | ROS2 WIT IMU 驱动（`/imu/data`） |
| `src/ds_imu_gps_localization` | ROS2 IMU+GPS EKF 融合（`/fused_path`） |
| `src/ds_gps_goal` | ROS2 经纬度 → Nav2 导航目标 |
| `src/ds_can_monitor` | CAN 监视（可选） |
| `docs/` | PROTOCOL_V3、STM32B 固件说明 |

## 依赖

- ROS2 Humble
- `ranger_ros2`、`ugv_sdk`（本地克隆到 `src/`，见 `.gitignore`）

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/agilexrobotics/ranger_ros.git ranger_ros2
```

## 编译

**Jetson 桥接：**

```bash
cd ~/catkin_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ds_jetson_bridge --symlink-install
source install/setup.bash
```

**GPS / IMU（可选）：**

```bash
colcon build --packages-select ds_gps_driver ds_imu_driver ds_imu_gps_localization ds_gps_goal --symlink-install
source install/setup.bash
```

启动 GPS / IMU：

```bash
ros2 launch ds_gps_driver gps_serial.launch.py port:=/dev/gps_usb
ros2 launch ds_imu_driver imu_serial.launch.py port:=/dev/imu_usb
```

详见 [docs/README.md §8.3](docs/README.md#83-ros2-移植总览方案-c已实现)。
