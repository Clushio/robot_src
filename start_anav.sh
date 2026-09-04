#!/bin/bash

set -e

# 加载 ROS2 Humble 基础环境
source /opt/ros/humble/setup.bash

# 加载当前工作空间
source /home/nav/anav_ros2_ws/install/setup.bash

# 进入 GUI 脚本目录
cd /home/nav/anav_ros2_ws/src/script

# 启动 GUI
exec python3 aNAV_ranger.py
