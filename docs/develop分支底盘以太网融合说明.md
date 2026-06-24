# develop 分支 · 底盘以太网融合说明

> 日期：2026-06-24（持续更新）  
> 适用：从 `peng` 个人分支切到 `develop` 后，以太网底盘 + GUI + Foxglove 联调  
> 环境：Jetson aarch64，Ubuntu 22.04，双网卡 `192.168.10.200` / `201`，launch 绑定 **`bind_ip:=192.168.10.201`**

---

## 1. 背景：两个版本的区别

| 项目 | `peng` 分支（旧） | `develop` 分支（现用） |
|------|-------------------|------------------------|
| 底盘业务（以太网） | Python 单节点 | **`agv_base_eth_bringe`**（UDP + 融合，单节点） |
| 以太网 UDP | 合并在业务节点内 | **`eth_gateway` 库**（`McuEthBridge`，默认不对外发 Topic） |
| GPS | 同包发布 `/fix` | 集成在 **`agv_base_eth_bringe`**（`gps_enable:=true`） |
| GUI | 同包 `cmd_vel_gui` | **`eth_gateway`** 包（entry: `cmd_vel_gui`） |
| 消息接口 | `jetson_can_msgs`（旧名） | **`jetson_mcu_msgs`** |
| 对外业务消息 | 分散多个 Topic | **`scr_sensor`**：`AgvControl` / `VehicleData` / `AgvFeatureStatus` |
| ROS 包名 | `agv_base_driver` | 目录 `agv_base_bringe/`，包名 **`agv_base_driver`** |

### 1.1 数据流（以太网 · 对外规范 Topic）

```text
cmd_vel_gui / Nav2
       ↓
/agv_control (scr_sensor/AgvControl)
       ↓
agv_base_eth_bringe（单 Python 节点）
  ├─ McuEthBridge：UDP BLOB ↔ MCU（进程内，默认不创建 /jetson_eth/*）
  └─ 融合发布：
       /Vehicle/VehicleData
       /Function/FeatureStatusInfo
       /odom + TF
       /fix（gps_enable=true）

RS232 链路：仍用 C++ agv_base_bringe_node（link_type=rs232）
Nav2：须发 /agv_control（cmd_vel_compat 已关闭）
联调旧协议：eth_gateway.launch.py + ros_uplink_publish:=true
```

### 1.2 对外业务 Topic

| Topic | 消息类型 | 说明 |
|-------|----------|------|
| `/agv_control` | `scr_sensor/AgvControl` | 速度/模式/灯光/清错；GUI 用 `use_twist_input=true` |
| `/Vehicle/VehicleData` | `scr_sensor/VehicleData` | 超声、电池、BMS、轮速、8 路电机、里程脉冲 |
| `/Function/FeatureStatusInfo` | `scr_sensor/AgvFeatureStatus` | 安全、链路、限速、模式、故障、灯光回馈 |
| `/odom` | `nav_msgs/Odometry` | 里程计 + TF |
| `/fix` | `sensor_msgs/NavSatFix` | GPS（`agv_base_eth_bringe` 内发布） |

**生产环境 `ros2 topic list` 不应再出现 `/jetson_eth/*`、`/cmd_vel`、`/vehicle/vehicle_data`。**

### 1.3 消息定义（`scr_sensor` 包）

| 消息文件 | 用途 |
|----------|------|
| `AgvControl.msg` | 下行：`mode_req`、运动（`v_mm_s` / `omega` / `steer` 或 `use_twist_input` + twist）、灯光、清错 |
| `VehicleData.msg` | 上行量测：超声、电池/BMS、四轮轮速/转角、8 路 `AgvMotorCompact`、里程脉冲 |
| `AgvFeatureStatus.msg` | 上行功能层：安全状态、链路 bit、限速、运动模式、故障码、灯光回馈、`jetson_seq` |
| `AgvMotorCompact.msg` | 单路电机（嵌在 `VehicleData.motors[8]`） |

`jetson_mcu_msgs`（`V3Command`、`V3Status`、BLOB 等）**仅 UDP 内部使用**，不再作为对外 ROS 接口。

### 1.4 Launch 架构变化（以太网）

| 阶段 | 启动方式 | 问题 |
|------|----------|------|
| 旧 | `eth_gateway` + C++ `agv_base_bringe_node` + `gps_rs232_to_fix` | `/jetson_eth/*` 全暴露，`ros2 topic list` 很乱 |
| **现** | 单节点 `agv_base_eth_bringe` + 可选 `cmd_vel_gui` | UDP 在进程内，对外 4+1 Topic |

Launch 文件：

- `agv_base_driver/launch/jetson_eth_bringup.launch.py` — 仅底盘
- `agv_base_driver/launch/jetson_eth_bringup_gui.launch.py` — 底盘 + GUI

---

## 2. 编译与启动（完整命令）

### 2.1 每次改代码后必须重新编译

改了 `.msg`、`agv_base_eth_bringe`、launch 或 `eth_gateway` 后，**必须** `colcon build` 再 `source`，否则运行的仍是旧 install。

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash

colcon build --symlink-install --allow-overriding eth_gateway \
  --packages-select scr_sensor agv_base_driver eth_gateway

source ~/catkin_ws/cangyirobot/install/setup.bash
```

首次恢复 develop 或缺包时，可编更多包：

```bash
colcon build --symlink-install --allow-overriding eth_gateway \
  --packages-select jetson_mcu_msgs scr_sensor eth_gateway gps_rs232_to_fix agv_base_driver
source install/setup.bash
```

### 2.2 启动底盘 + GUI（终端 1）

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash
export DISPLAY=:0

ros2 launch agv_base_driver jetson_eth_bringup_gui.launch.py \
  bind_ip:=192.168.10.201 mcu_ip:=192.168.10.30
```

仅底盘、无 GUI：

```bash
ros2 launch agv_base_driver jetson_eth_bringup.launch.py \
  bind_ip:=192.168.10.201 mcu_ip:=192.168.10.30
```

### 2.3 Foxglove Bridge（终端 2，可选）

```bash
cd ~/catkin_ws/cangyirobot
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Foxglove Studio：**连接类型选 Foxglove WebSocket**（不是 Rosbridge），URL：`ws://127.0.0.1:8765`。

### 2.4 验证 Topic

```bash
source ~/catkin_ws/cangyirobot/install/setup.bash
ros2 topic list
ros2 topic echo /Function/FeatureStatusInfo --once
ros2 topic echo /Vehicle/VehicleData --once
ros2 topic hz /odom
```

---

## 3. 调试问题与解决办法（实录）

### 3.1 GUI 显示「Jetson 链路丢失」，上行却有数据

**现象：**

- UDP 已连接，上行 BLOB（motion/mcu/sensor）正常
- `link_state = 1`（bit0 = Jetson 心跳丢失）
- `blob/mcu_status` 里 **`jetson_seq = 0`**（MCU 没收到下行）

**根因：** Jetson 双网卡同网段，TX 未绑定源 IP：

- `enp1s0f0` → `192.168.10.200`
- `enp1s0f1` → `192.168.10.201`（launch `bind_ip`）

旧 `udp_link.py`：RX 绑 `201:50002`，TX 从 `200` 发出，MCU 只认 `201`。

**修复：** `eth_gateway/eth_gateway/udp_link.py` 改为 **单 socket** 绑定 `(bind_ip, local_port)` 收发共用。

**验证（联调模式可见 mcu_status 时）：**

```bash
ros2 topic echo /jetson_eth/blob/mcu_status --once
# 期望 link_state: 0，jetson_seq 递增
```

临时路由（需 sudo）：

```bash
sudo ip route replace 192.168.10.0/24 dev enp1s0f1 src 192.168.10.201
```

---

### 3.2 切到 develop 后 `colcon build` 失败

| 现象 | 处理 |
|------|------|
| `rplidar_ros` 缺源码 | 从 `~/catkin_ws/backup/` 恢复，或 `--packages-skip rplidar_ros` |
| 缺 `eth_gateway`、`gps_rs232_to_fix` | 从 `peng` 恢复并适配 `jetson_mcu_msgs` |
| `vision_msgs`：`library_path.sh` not found | `colcon build --symlink-install --packages-select vision_msgs` |
| 删过 `build/` 后 install 损坏 | 重编相关包；必要时删 `install/` 中对应包残留再编 |

---

### 3.3 launch 起不来

| 错误 | 修复 |
|------|------|
| `No module named 'launch_ros.events.lifecycle.matchers'` | `from launch.events import matches_action` |
| `LifecycleNode.__init__() missing ... 'namespace'` | `LifecycleNode(..., namespace='')` |
| `No module named 'eth_gateway.mcu_eth_bridge'` | 已恢复 `eth_gateway/eth_gateway/mcu_eth_bridge.py` |
| `get_package_share_directory('eth_gateway')` 失败 | `colcon build --packages-select eth_gateway` |

---

### 3.4 `source install/setup.bash` 报错

多为某包 install 残缺（常见 `vision_msgs`）：

```bash
colcon build --symlink-install --packages-select vision_msgs
source install/setup.bash
```

**每个新终端**启动底盘或 Foxglove 前都要：

```bash
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash
```

---

### 3.5 Foxglove 连不上 / 看不到自定义消息

**问题 1：连接方式错误**

- 必须选 **Foxglove WebSocket**，URL `ws://127.0.0.1:8765`
- 选 Rosbridge 会连不上或类型不对

**问题 2：Bridge 未 source 工作空间**

Bridge 终端必须先 `source install/setup.bash`，否则报 `jetson_mcu_msgs not found` 或看不到 `scr_sensor` 消息。

**问题 3：Layout 仍订阅旧 Topic**

规范化后请改订阅：

- `/agv_control`、`/Vehicle/VehicleData`、`/Function/FeatureStatusInfo`、`/odom`
- 不要再用 `/vehicle/vehicle_data`、`/jetson_eth/*`

**问题 4：日志里 `Failed to retrieve parameters from /cmd_vel_gui`**

Tk GUI 对参数服务响应慢，Foxglove Bridge 会 WARN 并 ignore，**可忽略**，不影响看 Topic。

**安装（Jetson aarch64，用户本机 sudo）：**

```bash
# Foxglove Studio（deb）+ ros-humble-foxglove-bridge
sudo apt install ./foxglove-studio-latest-linux-arm64.deb
sudo apt install ros-humble-foxglove-bridge
```

---

### 3.6 `ros2 topic list` 里还有 `/jetson_eth/*`

**原因（历史）：** 旧架构下 `eth_gateway` 独立节点对外 publish 全部 `/jetson_eth/*`，与 `agv_base_bringe` 并行。

**现架构：** `agv_base_eth_bringe` 内 `McuEthBridge` 设 `ros_uplink_publish:=false`，**不创建** `/jetson_eth/*` publisher。

**若仍看到旧 Topic：**

1. 未重新编译 / 未 `source` 新 install
2. 仍在跑旧 launch（独立 `eth_gateway` + C++ bringe）
3. 另一个终端还开着旧进程 → `pkill -f agv_base|eth_gateway|cmd_vel_gui` 后重启

**规范模式下期望列表：**

```text
/agv_control
/Vehicle/VehicleData
/Function/FeatureStatusInfo
/odom
/fix
/tf
/rosout
/parameter_events
（+ GUI / Foxglove 临时 topic，如 /clicked_point）
```

**需要看原始 BLOB 联调时：**

```bash
ros2 launch eth_gateway eth_gateway.launch.py ros_uplink_publish:=true
```

---

### 3.7 Topic 规范化与消息融合（方案 A）

**目标：** 对外只有 4 个业务 Topic + `/fix`，旧 `jetson_mcu_msgs` 仅作 UDP 内部协议。

| 旧接口 | 现接口 |
|--------|--------|
| `/cmd_vel` + `/agv/light_enable` + `/jetson_eth/command` | `/agv_control`（`AgvControl`） |
| `/vehicle/vehicle_data` | `/Vehicle/VehicleData` |
| `/jetson_eth/v3_status` + `blob/motion` 等安全字段 | `/Function/FeatureStatusInfo` |
| 分散 BLOB（motor/energy/…） | 字段并入 `VehicleData` / `AgvFeatureStatus` |

**说明：** ROS 中 **一个 Topic 只有一种 Message**；融合方式是扩展字段，不是多种 Message 共用一个 Topic。

**Nav2：** `cmd_vel_compat` 已默认 **false**，导航须改发 `/agv_control`（`AgvControl`，或 `use_twist_input=true` + twist 字段）。`nav_manager` 仍发 `/cmd_vel` 时底盘不会动。

**Nav2 / nav_manager 现状（待改）：**

- `nav_manager` 仍发布 `geometry_msgs/Twist` → `/cmd_vel`（见 `nav_manager_params.yaml` 的 `chassis_cmd_vel_topic`）
- 以太网底盘**不再订阅** `/cmd_vel`
- 后续需改为发布 `scr_sensor/AgvControl` → `/agv_control`，或增加 relay 节点（未做）

---

### 3.10 Foxglove Bridge 日志里「还有旧 Topic 名」

**现象：** Bridge 终端前半段出现 `/vehicle/vehicle_data`、`/cmd_vel`、`/jetson_eth/*`，后半段才有 `/agv_control`、`/Vehicle/VehicleData`。

**原因：**

1. 日志是**两次 launch 会话累积**（改 Topic 前启动过一次 Bridge）
2. 旧进程未停干净，ROS 图里短暂并存新旧 publisher
3. Foxglove **Layout 缓存**仍订阅旧 channel

**处理：**

- 停掉旧 launch → 重新编译 `source` → 只起新 `jetson_eth_bringup_gui`
- Foxglove 里新建 Layout，手动选新 Topic
- 确认 Bridge 终端已 `source install/setup.bash`

---

### 3.11 旧 launch 与 orphan 进程

**现象：** 只起了 GUI，但 `eth_gateway` 仍在跑（PPID=1），或 `ros2 node list` 有僵尸节点名。

**原因：** 之前 `Ctrl+C` 只杀了 launch 父进程，UDP 线程或 Tk GUI 子进程未退出。

**处理：**

```bash
pkill -f "jetson_eth_bringup|agv_base_eth_bringe|agv_base_bringe|cmd_vel_gui|eth_gateway"
ros2 daemon stop
sleep 2
ros2 node list   # 应为空或只剩你要的节点
```

---

### 3.12 C++ bringe 与 Python eth bringe 分工

| 节点 | 可执行文件 | 链路 | 说明 |
|------|-----------|------|------|
| `agv_base_eth_bringe` | Python | **eth** | 生产默认；UDP + 融合 + 4 Topic |
| `agv_base_bringe_node` | C++ lifecycle | **rs232** 等 | 仍订阅 `/jetson_{link}/*` 或旧参数，eth 链路**勿再使用** |

C++ 节点里曾做过一版 Topic 规范化（订阅 `/agv_control`、发布 `/Vehicle/VehicleData`），以太网已改由 Python 单节点接管。

---

### 3.8 磁盘满 / 清理

Jetson 磁盘满会导致编译失败、日志暴涨。可清理：

```bash
rm -rf ~/.ros/log/*
# 删除已安装的 deb 包、解压目录等（按实际情况）
df -h
```

---

### 3.9 停止所有底盘相关节点

```bash
pkill -f "jetson_eth_bringup|agv_base_eth_bringe|agv_base_bringe|cmd_vel_gui|eth_gateway"
ros2 daemon stop
```

---

## 4. 包与节点速查（当前）

| ROS 包 | 可执行文件 | 节点名 | 用途 |
|--------|-----------|--------|------|
| `eth_gateway` | `agv_base_eth_bringe` | `agv_base_bringe` | **以太网生产节点**（UDP + 4 Topic + /fix） |
| `eth_gateway` | `cmd_vel_gui` | `cmd_vel_gui` | Tk 遥控 GUI → `/agv_control` |
| `eth_gateway` | `eth_gateway` | `eth_gateway` | **仅联调**（`ros_uplink_publish:=true`） |
| `agv_base_driver` | `agv_base_bringe_node` | `agv_base_bringe` | RS232 等 C++ lifecycle 节点 |
| `gps_rs232_to_fix` | `gps_to_fix` | `gps_to_fix` | 旧 GPS 节点（以太网已集成，可不启） |

主要源码：

- `src/drivers/eth_gateway/eth_gateway/agv_base_eth_bringe_node.py` — 以太网生产节点
- `src/drivers/eth_gateway/eth_gateway/mcu_eth_bridge.py` — UDP BLOB + TimeSync
- `src/drivers/eth_gateway/eth_gateway/chassis_fusion.py` — 缓存 → VehicleData / FeatureStatus
- `src/drivers/eth_gateway/eth_gateway/gps_fix.py` — GPS → `/fix`
- `src/drivers/eth_gateway/eth_gateway/blob_topic_pub.py` — `ros_uplink_publish` 开关
- `src/drivers/eth_gateway/eth_gateway/udp_link.py` — 单 socket 绑定源 IP
- `src/drivers/eth_gateway/eth_gateway/cmd_vel_gui_node.py` — GUI → `/agv_control`
- `src/interfaces/scr_sensor/msg/*.msg` — 对外业务消息

---

## 9. 本次代码改动清单（便于同事 review）

### 9.1 新增

| 路径 | 说明 |
|------|------|
| `scr_sensor/msg/AgvControl.msg` | 统一下行控制 |
| `scr_sensor/msg/AgvFeatureStatus.msg` | 功能/安全状态 |
| `scr_sensor/msg/AgvMotorCompact.msg` | 电机紧凑体 |
| `eth_gateway/agv_base_eth_bringe_node.py` | 以太网单节点（UDP + 发布） |
| `eth_gateway/chassis_fusion.py` | BLOB 缓存融合 |
| `eth_gateway/gps_fix.py` | 集成 GPS `/fix` |
| `eth_gateway/mcu_eth_bridge.py` | 从 git 历史恢复并适配 `jetson_mcu_msgs` |

### 9.2 修改

| 路径 | 说明 |
|------|------|
| `scr_sensor/msg/VehicleData.msg` | 扩展 BMS/电机/里程；去掉安全字段（迁至 FeatureStatus） |
| `blob_topic_pub.py` | 增加 `enable_ros_publish`，false 时不创建 `/jetson_eth/*` |
| `mcu_eth_bridge.py` | 参数 `ros_uplink_publish` / `ros_link_publish` 等 |
| `udp_link.py` | 单 socket，修复双网卡 TX 源 IP |
| `cmd_vel_gui_node.py` | 发 `/agv_control`；状态读 VehicleData + FeatureStatus |
| `jetson_eth_bringup*.launch.py` | 改为只起 `agv_base_eth_bringe`，不再 include 独立 eth_gateway |
| `agv_base_bringe_params.yaml` | `cmd_vel_compat_enable: false` |
| `eth_gateway/setup.py` | 增加 entry `agv_base_eth_bringe` |

### 9.3 废弃 / 不再作为对外接口

| 旧 Topic | 替代 |
|----------|------|
| `/jetson_eth/*` | 进程内 UDP，联调时 `ros_uplink_publish:=true` |
| `/vehicle/vehicle_data` | `/Vehicle/VehicleData` |
| `/agv/light_enable` | `AgvControl.light_*` |
| `/cmd_vel`（底盘侧） | `/agv_control` |

### 9.4 待办（未在本轮完成）

- [ ] `nav_manager` 改发 `/agv_control`（`AgvControl`）
- [ ] Nav2 controller 输出 → `/agv_control` relay 或直改
- [ ] 与同事确认 `AgvFeatureStatus` 消息名 vs Topic 路径 `/Function/FeatureStatusInfo` 是否最终定稿

---

## 5. 常见「没数据」说明

| Topic / 现象 | 原因 |
|--------------|------|
| `/fix` 经纬度 nan | 室内无 GPS 星，`NO_FIX` 正常 |
| `/odom` 不动 | 底盘未收到 `/agv_control` 或 MCU 未反馈速度 |
| Nav2 不动 | 仍发 `/cmd_vel`，需改 `/agv_control` |
| `/imu/data`、`/scan` | 未 launch IMU/雷达 |
| Foxglove 无自定义 msg | Bridge 终端未 `source install/setup.bash` |
| GUI 安全状态 WAIT | 等 `/Function/FeatureStatusInfo`；确认 `agv_base_eth_bringe` 在跑且链路 UP |

---

## 6. 文档与 Git 找回

`develop` 的 `docs/` 曾几乎只剩 README；完整旧文档在 **`peng` 分支**。

```bash
# 查看 peng 上某文档
git show peng:docs/以太网接入与联调.md | less

# 恢复整个 docs（确认后再 commit）
git checkout peng -- docs/

# 对比 develop / peng 文档差异
git diff peng develop -- docs/
```

---

## 7. ros_network_viz 说明

**只能看连接图，不能 echo 消息。** 看消息用：

```bash
ros2 topic echo /Vehicle/VehicleData --once
ros2 topic hz /odom
rqt_topic   # 若已安装
# 或 Foxglove Studio
```

network viz 启动：

```bash
source ~/viz_ws/install/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash
ros2 run ros_network_viz ros_network_viz
```

---

## 8. 变更记录

| 日期 | 说明 |
|------|------|
| 2026-06-24 | develop 融合：恢复 eth_gateway/gps；修复 UDP 源 IP；统一 jetson_mcu_msgs；修复 launch |
| 2026-06-24 | Topic 规范化：AgvControl / VehicleData / AgvFeatureStatus；GUI 改 `/agv_control` |
| 2026-06-24 | **单节点架构**：`agv_base_eth_bringe` 内嵌 McuEthBridge，关闭对外 `/jetson_eth/*`；补全 msg 字段；更新 launch / GUI |
| 2026-06-24 | 文档：补充调试实录、完整编译启动命令、代码改动清单、nav_manager 待办、Foxglove 旧 Topic 误解说明 |
