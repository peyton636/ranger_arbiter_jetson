# agx_arm_msgs  
GripperRawStatus: 夹爪底层状态
GripperCommand: 夹爪控制命令
GripperStatus: 夹持状态与结果
AgxArmStatus: 机械臂整体状态（控制模式、运行状态、示教状态、关节健康）
GripperStatus: 夹爪开口宽度、夹持力及驱动器健康状态

# algo_msgs


# cali_msgs


# can_msgs


# geometry_msgs
ROS2 robot built-in msgs file

# diagnostic_msgs
ROS2 robot built-in msgs file

# map_msgs
ROS2 robot built-in msgs file

# nav_msgs 
ROS2 robot built-in msgs file

# orbbec_camera_msgs
orbbec_camera_msg

# scr_sensor


# sensor_msgs
ROS2 robot built-in msgs file

# std_msgs
ROS2 robot built-in msgs file

# trajectory_msgs
ROS2 robot built-in msgs file

# vision_msgs

- 编译ROS2 Package 包， 用于ROS2工具以及可视化等。
```bash
source /opt/ros/humble/setup.bash 
# geometry_msgs, diagnostic_msgs, map_msgs, nav_msgs, sensor_msgs, std_msgs, trajectory_msgs 跳过本地编译，使用ros hubhub环境中的msg
colcon build --packages-skip std_msgs sensor_msgs geometry_msgs diagnostic_msgs map_msgs nav_msgs trajectory_msgs
```

# Msg 命名限制
1. 为了兼容ros2 topic， 需要先创建 ros msg 的定义，再转换为idl （直接创建一个新的idl可能无法兼容ros2）
2. ros msg的namespace也就是目录结构必须符合  aa_bb/msg/cc.msg 的方式
3. 新加的msg需同时添加 ros2 package 所需要的package.xml 以及 CMakeLists.txt 文件
4. 新增的msg需要符合命令规范，命名方式参考已有的msg，不然可能导致无法转为idl

1. 历史问题， MiddleAppErrMsg.idl 需要手动添加 bitmask MiddleAppErrMask 。
2. 使用idl2cpp.sh生成 generated ， 注意使用Depends中的的FastddsGen的版本，否则会生成大量不一样的代码。