# ranger_arbiter_jetson

Jetson ↔ STM32B ↔ Ranger Mini 联调工程（ROS2 Humble + Protocol V3）。

## 文档

详见 [docs/README.md](docs/README.md)（编译、串口接线、RS232 数据流、联调步骤）。

## 主要包

| 包 | 说明 |
|----|------|
| `src/ds_jetson_bridge` | Jetson V3 串口桥（`/cmd_vel` ↔ 24B UART） |
| `src/ds_serial_monitor` | 串口监视 / pl2303 驱动辅助 |
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

```bash
cd ~/catkin_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ds_jetson_bridge --symlink-install
source install/setup.bash
```
