#!/usr/bin/env bash
# 仓颉机器人统一 ROS 2 网络环境。
# 开发机与机器人（192.168.10.200）须 source 同一份配置，否则无法互相发现 topic。
#
# 用法:
#   source "$(ros2 pkg prefix cangyi_bringup)/share/cangyi_bringup/scripts/ros_env.sh"
# 或工作区:
#   source src/bringup/cangyi_bringup/scripts/ros_env.sh

# 固定 domain，可通过外部环境变量覆盖（便于临时隔离调试）
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-40}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
