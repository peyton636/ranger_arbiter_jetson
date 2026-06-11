# bringup

放 launch、参数 YAML、RViz 配置、系统组合启动。
原则：系统集成放这里，不污染功能包。

## 已提供程序

- ROS 2 bringup 包: `cangyi_bringup`
- 主启动文件: `cangyi_bringup/launch/bringup.launch.py`

## 快速使用

```bash
cd ~/workspace/cangyirobot
colcon build --symlink-install --packages-select cangyi_bringup
source install/setup.bash

# 仅控制
ros2 launch cangyi_bringup bringup.launch.py mode:=control

# 控制 + RViz
ros2 launch cangyi_bringup bringup.launch.py mode:=rviz

# 控制 + MoveIt
ros2 launch cangyi_bringup bringup.launch.py mode:=moveit
```