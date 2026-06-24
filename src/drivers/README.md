## drivers

| 目录/包名 | ROS 包名 | 可执行文件 | 作用 |
|-----------|----------|------------|------|
| `agv_base_bringe/` | `agv_base_driver` | `agv_base_eth_bringe`, `cmd_vel_gui`, `eth_gateway` | 以太网底盘 UDP + 业务 Topic + GUI + `/fix` |

**以太网一键启动：**
```bash
ros2 launch agv_base_driver jetson_eth_bringup_gui.launch.py \
  bind_ip:=192.168.10.201 bind_device:=enp1s0f1 mcu_ip:=192.168.10.30
```

原则：只做设备接入、协议解析、基础状态发布，不写业务逻辑。
