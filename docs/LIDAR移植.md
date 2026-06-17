# RPLIDAR S2 ROS2 移植与联调

| 元数据 | 值 |
|--------|-----|
| **文档版本** | v1.0 |
| **日期** | 2026-06-12 |
| **雷达型号** | RPLIDAR **S2**（思岚，CP2102 USB 转接器，波特率 **1 Mbps**） |
| **工作区** | `~/catkin_ws/cangyirobot` |
| **驱动包路径** | `src/drivers/ds_lidar_driver/rplidar_ros-ros2/`（ROS2 包名 `rplidar_ros`） |
| **厂商手册** | S2 开发套装使用手册 rev.1.3 |

本文档说明如何把厂商 `rplidar_ros`（ROS2 分支）接入本工程，并在 Jetson 上验证 `/scan` 数据。

---

## 1. 目录结构

```text
src/drivers/ds_lidar_driver/
├── rplidar_ros-ros2/     ← ROS2 包（编译用这个，SDK 已在 sdk/ 子目录）
└── rplidar_sdk-master/   ← 独立 SDK 源码，ROS 编译不依赖，可忽略
```

---

## 2. 硬件连接（S2 开发套装）

S2 USB 转接器背面有 **两个 USB 口**，作用不同：

| 接口 | 线材 | 作用 | 接哪里 |
|------|------|------|--------|
| **Micro-USB** | 套装 Micro-USB 线 | **数据**：CP2102 串口通信（1 Mbps） | Jetson USB（出现 `/dev/ttyUSBx`） |
| **USB-DC 电源口** | 套装电源线 | **供电**：5V DC，给测距核心 + 电机 | 5V 电源（手册推荐接电源适配器） |

**两根线都需要接好**，雷达才能转起来并输出扫描数据。只插 Micro-USB、不接电源时：转接器指示灯可能亮，但**电机不转、无 `/scan`**。

### 2.1 两个口都插在 Jetson 上可以吗？

可以，但需注意 **供电电流**：

- **Micro-USB（数据）**：必须插 Jetson，用于串口通信。
- **USB-DC（供电）**：插 Jetson 的另一个 USB 口时，由 Jetson USB 口提供 5V。一般能点亮、能通信；若电机不转或频繁掉线，多半是 **USB 供电电流不足**（S2 电机启动电流较大）。
- **推荐**：供电口接 **独立 5V 电源适配器**（手册原意）；数据口仍接 Jetson。若现场只能都用 Jetson，优先插 **带供电的 USB Hub** 或功耗更大的 USB 口。

### 2.2 本机串口识别

S2 转接器芯片为 **CP2102**（`10c4:ea60`），在 Linux 下设备名通常为 `ttyUSBx`。

```bash
ls -la /dev/serial/by-id/
# 示例：usb-Silicon_Labs_CP2102N_... -> ../../ttyUSB7
```

| 检查项 | 命令 | 期望 |
|--------|------|------|
| USB 识别 | `lsusb \| grep -i "10c4:ea60\|CP210"` | 有 Silicon Labs CP210x |
| 内核驱动 | `lsmod \| grep cp210x` | 有 `cp210x` 模块 |
| 串口权限 | `groups` | 用户在 `dialout` 组；或 `ls -l /dev/ttyUSB7` 可读写 |

---

## 3. 是否需要安装驱动？

### 3.1 Linux / Jetson（本机）— **一般不需要**

手册里的 CP2102 驱动安装是针对 **Windows**（`CP210x VCP Windows`）。

在 **Ubuntu / Jetson（Linux 内核）** 上，CP2102 由内核自带 **`cp210x`** 驱动支持，插入 USB 后自动出现 `/dev/ttyUSBx`，**无需**再装 Windows 驱动或厂商 `.inf` 文件。

验证：

```bash
lsmod | grep cp210x
dmesg | tail -20 | grep -i cp210
ls -la /dev/serial/by-id/ | grep -i CP210
```

若 `lsusb` 能看到 CP2102 但没有 `ttyUSB`，再排查内核模块（极少见）：

```bash
sudo modprobe cp210x
```

### 3.2 仍需配置的（不是“装驱动”）

| 项目 | 说明 |
|------|------|
| **串口权限** | 用户加入 `dialout` 组，或 `sudo chmod 666 /dev/ttyUSBx` |
| **udev 固定别名（可选）** | 安装 `rplidar.rules` 后可用 `/dev/rplidar` |
| **ROS2 依赖** | `rclcpp`、`sensor_msgs` 等随 Humble 已有 |

安装 udev 规则（可选，固定为 `/dev/rplidar`）：

```bash
cd ~/catkin_ws/cangyirobot/src/drivers/ds_lidar_driver/rplidar_ros-ros2
sudo cp scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/rplidar
```

---

## 4. 编译

每个新终端先：

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash
```

首次或改代码后编译：

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
colcon build --packages-select rplidar_ros --symlink-install
source install/setup.bash
```

---

## 5. 启动与验证（RPLIDAR S2）

S2 专用参数：

| 参数 | 值 |
|------|-----|
| launch 文件 | `rplidar_s2_launch.py` |
| 波特率 | **1000000**（1 Mbps） |
| 默认扫描模式 | `DenseBoost` |
| 发布话题 | `/scan`（`sensor_msgs/LaserScan`） |

### 5.1 终端 1 — 启动雷达节点

将 `serial_port` 换成你本机 CP2102 对应端口（示例为 `ttyUSB7`）：

```bash
source ~/catkin_ws/cangyirobot/install/setup.bash

ros2 launch rplidar_ros rplidar_s2_launch.py \
  serial_port:=/dev/ttyUSB7 \
  serial_baudrate:=1000000
```

使用稳定 by-id 路径（推荐，避免重启后 tty 号变化）：

```bash
export LIDAR_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2a5962c3ec7def118ff1201cedd322a4-if00-port0

ros2 launch rplidar_ros rplidar_s2_launch.py \
  serial_port:=$LIDAR_PORT
```

若已配置 udev：

```bash
ros2 launch rplidar_ros rplidar_s2_launch.py serial_port:=/dev/rplidar
```

**成功标志：**

- 终端打印 `RPLIDAR running on ROS2 package rplidar_ros`
- 雷达电机开始旋转（需供电口已接好）

### 5.2 终端 2 — 验证数据

```bash
source ~/catkin_ws/cangyirobot/install/setup.bash

# 话题是否存在
ros2 topic list | grep scan

# 发布频率（应有稳定 Hz）
ros2 topic hz /scan

# 看一帧数据
ros2 topic echo /scan --once
```

**通过条件：**

- `ros2 topic hz /scan` 有稳定频率
- `echo` 中 `ranges: [...]` 有有效数值（非全 0 / 全 inf）

### 5.3 可视化（可选，需桌面 / 显示器）

```bash
ros2 launch rplidar_ros view_rplidar_s2_launch.py serial_port:=/dev/ttyUSB7
```

RViz 中应能看到一圈激光扫描点。

---

## 6. 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 电机不转 | 未接 5V 供电口，或 Jetson USB 供电不足 | 接电源适配器；或换带供电 Hub |
| `Permission denied` | 串口权限 | `sudo chmod 666 /dev/ttyUSBx` 或装 udev |
| 节点起来但无 `/scan` | 用了 A1 等错误 launch / 波特率不对 | 必须用 `rplidar_s2_launch.py`，波特率 `1000000` |
| 串口打不开 | 端口选错（连到 5G 模组等） | `ls -la /dev/serial/by-id/`，选 CP2102 那条 |
| 数据抖动大 | 刚上电未预热 | 手册建议扫描运行约 **2 分钟** 后精度最佳 |
| 找不到包 | 未 source install | `source ~/catkin_ws/cangyirobot/install/setup.bash` |

---

## 7. 命令速查

```bash
# 环境
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash

# 编译
cd ~/catkin_ws/cangyirobot
colcon build --packages-select rplidar_ros --symlink-install
source install/setup.bash

# 查串口
ls -la /dev/serial/by-id/ | grep -i CP210

# 启动 S2
ros2 launch rplidar_ros rplidar_s2_launch.py serial_port:=/dev/ttyUSB7

# 验证
ros2 topic hz /scan
ros2 topic echo /scan --once

# RViz（可选）
ros2 launch rplidar_ros view_rplidar_s2_launch.py serial_port:=/dev/ttyUSB7
```

---

## 8. 后续集成（待做）

当前为厂商包直接验证阶段。后续可在 `ds_lidar_driver` 下封装本团队 launch、固定 `frame_id`、与 `TOPIC_NAMING.md` 对齐，并纳入整车 launch。
