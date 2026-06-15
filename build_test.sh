#!/usr/bin/env bash

rm -rf build log install
colcon build --symlink-install --packages-select yolo_detector
source install/setup.bash
# ros2 launch yolo_detector classifier.launch.py &
# ros2 launch yolo_detector detector.launch.py &
# ros2 launch yolo_detector obb.launch.py &
ros2 launch yolo_detector pose.launch.py &
ros2 launch yolo_detector segmentor.launch.py
